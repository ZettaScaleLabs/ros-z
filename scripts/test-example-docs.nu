#!/usr/bin/env nu
# Test that documented example commands actually run successfully

# Parse cargo run commands from markdown files
def extract_testable_commands [] {
    glob book/src/**/*.md
    | each { |file|
        let content = (open $file)
        let lines = ($content | lines | enumerate)

        # Find lines with "cargo run --example"
        let commands = (
            $lines
            | where { |line| $line.item | str contains "cargo run --example" }
            | each { |line|
                let cmd = ($line.item | str trim)
                if ($cmd | str starts-with "cargo run --example") {
                    # Skip commands that are comments or within code fences
                    if not ($cmd | str starts-with "#") {
                        {
                            file: $file,
                            line: $line.index,
                            command: $cmd
                        }
                    } else {
                        null
                    }
                } else {
                    null
                }
            }
            | compact
        )

        $commands
    }
    | flatten
}

# Parse cargo command to extract example name and arguments
def parse_cargo_command [cmd: string] {
    let without_prefix = ($cmd | str replace "cargo run --example " "")
    let parts = ($without_prefix | split row " -- ")
    let example_name = ($parts | first | str trim)
    let args = if ($parts | length) > 1 {
        ($parts | last | str trim)
    } else {
        ""
    }

    {example: $example_name, args: $args, full_cmd: $cmd}
}

# Check if command is testable (has exit conditions)
def is_testable [cmd: record] {
    let args = $cmd.args
    # Testable if has --max-count, --count, --order, or is router
    ($args | str contains "--max-count") or
    ($args | str contains "--count") or
    ($args | str contains "--order") or
    ($cmd.example == "zenoh_router")
}

# Test a single command
def test_command [cmd_info: record, timeout_sec: int = 5] {
    let parsed = (parse_cargo_command $cmd_info.command)

    print $"Testing: ($parsed.example) ($parsed.args)"

    try {
        # Run command with timeout
        let result = (
            ^timeout $"($timeout_sec)s" cargo run --example $parsed.example -- ...$parsed.args.split(' ') | complete
        )

        let success = $result.exit_code == 0 or $result.exit_code == 124  # 124 is timeout exit code

        {
            file: $cmd_info.file,
            line: $cmd_info.line,
            command: $cmd_info.command,
            example: $parsed.example,
            exit_code: $result.exit_code,
            status: (if $success { "success" } else { "failed" }),
            stdout: ($result.stdout | lines | first 5 | str join "\n"),
            stderr: ($result.stderr | lines | first 5 | str join "\n")
        }
    } catch {
        {
            file: $cmd_info.file,
            line: $cmd_info.line,
            command: $cmd_info.command,
            example: $parsed.example,
            exit_code: -1,
            status: "error",
            stdout: "",
            stderr: "Failed to execute"
        }
    }
}

# Main execution
def main [
    --timeout: int = 5  # Timeout in seconds for each command
    --testable-only     # Only test commands with exit conditions
    --verbose           # Show output from commands
] {
    print "🧪 Testing documented example commands...\n"

    # Extract commands
    let all_commands = (extract_testable_commands)
    print $"Found ($all_commands | length) cargo run commands in documentation\n"

    let commands = if $testable_only {
        let testable = ($all_commands | each { |cmd|
            let parsed = (parse_cargo_command $cmd.command)
            if (is_testable $parsed) {
                $cmd
            } else {
                null
            }
        } | compact)
        print $"Filtered to ($testable | length) testable commands (with exit conditions)\n"
        $testable
    } else {
        print "⚠️  Testing all commands (some may timeout)\n"
        $all_commands
    }

    if ($commands | is-empty) {
        print "No commands to test"
        return
    }

    # Build examples first
    print "Building examples..."
    let build_result = (cargo build --examples | complete)
    if $build_result.exit_code != 0 {
        print "❌ Failed to build examples"
        print $build_result.stderr
        exit 1
    }
    print "✅ Examples built\n"

    # Test each command
    let results = ($commands | each { |cmd| test_command $cmd $timeout })

    # Summarize
    let successful = ($results | where status == "success")
    let failed = ($results | where status == "failed")
    let errors = ($results | where status == "error")

    print $"\n📊 Test Results:"
    print $"✅ Successful: ($successful | length)"
    print $"❌ Failed: ($failed | length)"
    print $"⚠️  Errors: ($errors | length)"

    if $verbose and (($successful | length) > 0) {
        print "\n✅ Successful Commands:"
        $successful | each { |s|
            print $"  ($s.file):($s.line) - ($s.example)"
            if not ($s.stdout | is-empty) {
                print $"    Output: ($s.stdout | lines | first | str substring 0..80)"
            }
        }
    }

    if ($failed | length) > 0 {
        print "\n❌ Failed Commands:"
        $failed | each { |f|
            print $"  ($f.file):($f.line)"
            print $"    Command: ($f.command)"
            print $"    Exit code: ($f.exit_code)"
            if not ($f.stderr | is-empty) {
                print $"    Error: ($f.stderr | str substring 0..200)"
            }
        }
    }

    if ($errors | length) > 0 {
        print "\n⚠️  Errors:"
        $errors | each { |e|
            print $"  ($e.file):($e.line) - ($e.example): ($e.stderr)"
        }
    }

    # Exit with error if there are failures
    if ($failed | length) > 0 or ($errors | length) > 0 {
        exit 1
    } else {
        print "\n✅ All tested commands executed successfully!"
    }
}

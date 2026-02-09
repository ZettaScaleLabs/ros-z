#!/usr/bin/env nu
# Validate CLI documentation against actual command-line help

# Parse cargo run commands from markdown files
def extract_cargo_commands [] {
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
                    {
                        file: $file,
                        line: $line.index,
                        command: $cmd
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
    # Remove cargo run --example prefix
    let without_prefix = ($cmd | str replace "cargo run --example " "")

    # Split on --
    let parts = ($without_prefix | split row " -- ")
    let example_name = ($parts | first | str trim)
    let args = if ($parts | length) > 1 {
        ($parts | last | str trim)
    } else {
        ""
    }

    {example: $example_name, args: $args}
}

# Validate a single command
def validate_command [cmd_info: record] {
    let parsed = (parse_cargo_command $cmd_info.command)

    # Try to get help for the example
    try {
        let help_result = (cargo run --example $parsed.example -- --help | complete)

        if $help_result.exit_code != 0 {
            {
                file: $cmd_info.file,
                line: $cmd_info.line,
                command: $cmd_info.command,
                example: $parsed.example,
                status: "error",
                message: "Example failed to run"
            }
        } else {
            # Parse arguments from the command
            let doc_args = ($parsed.args | split row " " | where { |a| $a | str starts-with "--" } | each { |a| $a | split row "=" | first | str trim })

            # Extract valid flags from help text
            let help_lines = ($help_result.stdout | lines)
            let valid_flags = (
                $help_lines
                | where { |line| $line | str trim | str starts-with "-" }
                | each { |line|
                    let parts = ($line | str trim | split row " " | first)
                    if ($parts | str contains ",") {
                        # Has short and long form: -a, --arg
                        $parts | split row "," | last | str trim
                    } else {
                        $parts | str trim
                    }
                }
                | where { |f| $f | str starts-with "--" }
            )

            # Check if documented args are valid
            let invalid_args = (
                $doc_args
                | where { |arg|
                    not ($arg in $valid_flags)
                }
            )

            if ($invalid_args | is-empty) {
                {
                    file: $cmd_info.file,
                    line: $cmd_info.line,
                    command: $cmd_info.command,
                    example: $parsed.example,
                    status: "valid",
                    message: null
                }
            } else {
                {
                    file: $cmd_info.file,
                    line: $cmd_info.line,
                    command: $cmd_info.command,
                    example: $parsed.example,
                    status: "invalid_args",
                    message: $"Invalid flags: ($invalid_args | str join ', ')"
                }
            }
        }
    } catch {
        {
            file: $cmd_info.file,
            line: $cmd_info.line,
            command: $cmd_info.command,
            example: $parsed.example,
            status: "error",
            message: "Failed to validate"
        }
    }
}

# Main execution
def main [
    --verbose  # Show all commands, not just issues
] {
    print "🔍 Validating CLI documentation...\n"

    # Extract all cargo commands from docs
    let commands = (extract_cargo_commands)
    print $"Found ($commands | length) cargo run commands in documentation\n"

    if ($commands | is-empty) {
        print "✅ No commands to validate"
        return
    }

    # Validate each command
    print "Validating commands...\n"
    let results = ($commands | each { |cmd|
        print $"Checking: ($cmd.command | str substring 0..60)..."
        validate_command $cmd
    })

    # Summarize results
    let valid = ($results | where status == "valid")
    let invalid = ($results | where status == "invalid_args")
    let errors = ($results | where status == "error")

    print $"\n📊 Validation Results:"
    print $"✅ Valid: ($valid | length)"
    print $"⚠️  Invalid arguments: ($invalid | length)"
    print $"❌ Errors: ($errors | length)"

    if $verbose and (($valid | length) > 0) {
        print "\n✅ Valid Commands:"
        $valid | each { |v|
            print $"  ($v.file):($v.line) - ($v.example)"
        }
    }

    if ($invalid | length) > 0 {
        print "\n⚠️  Invalid Arguments:"
        $invalid | each { |inv|
            print $"  ($inv.file):($inv.line)"
            print $"    Command: ($inv.command)"
            print $"    Issue: ($inv.message)\n"
        }
    }

    if ($errors | length) > 0 {
        print "\n❌ Errors:"
        $errors | each { |err|
            print $"  ($err.file):($err.line)"
            print $"    Command: ($err.command)"
            print $"    Error: ($err.message)\n"
        }
    }

    # Exit with error code if there are issues
    if ($invalid | length) > 0 or ($errors | length) > 0 {
        exit 1
    } else {
        print "\n✅ All documented commands are valid!"
    }
}

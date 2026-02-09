#!/usr/bin/env nu
# Check documentation coverage for examples
# This script parses Cargo.toml for [[example]] entries and scans docs for references

# Extract example names from ros-z/Cargo.toml
def extract_examples [] {
    # Read file as raw string, then split by [[example]] sections
    let content = (cat ros-z/Cargo.toml)
    let example_names = (
        $content
        | split row "[[example]]"
        | skip 1
        | each { |section|
            # Extract name = "..." line
            let name_line = (
                $section
                | lines
                | where { |line| $line | str starts-with "name = \"" }
                | first
            )
            if ($name_line | is-empty) {
                null
            } else {
                $name_line | str replace 'name = "' '' | str replace '"' '' | str trim
            }
        }
        | compact
    )

    $example_names
}

# Check if an example is documented in mdbook
def check_example_docs [example_name: string] {
    # Search for the example name in book/src/**/*.md files
    let matches = (
        glob book/src/**/*.md
        | each { |file|
            let content = (open $file)
            let has_cargo_run = ($content | str contains $"cargo run --example ($example_name)")
            let has_code_include = ($content | str contains $"demo_nodes/($example_name).rs")
            let has_mention = ($content | str contains $example_name)

            if $has_cargo_run or $has_code_include {
                {file: $file, level: "documented", type: "detailed"}
            } else if $has_mention {
                {file: $file, level: "mentioned", type: "brief"}
            } else {
                null
            }
        }
        | compact
    )

    if ($matches | is-empty) {
        {example: $example_name, status: "undocumented", files: []}
    } else {
        let level = if ($matches | any { |m| $m.level == "documented" }) {
            "documented"
        } else {
            "mentioned"
        }
        {example: $example_name, status: $level, files: ($matches | get file)}
    }
}

# Main execution
def main [] {
    print "📊 Example Documentation Coverage Check"
    print "========================================\n"

    # Extract all examples
    let examples = (extract_examples)
    let total = ($examples | length)

    print $"Total examples found: ($total)\n"

    # Check each example
    let results = ($examples | each { |ex| check_example_docs $ex })

    # Categorize results
    let documented = ($results | where status == "documented")
    let mentioned = ($results | where status == "mentioned")
    let undocumented = ($results | where status == "undocumented")

    let doc_count = ($documented | length)
    let mention_count = ($mentioned | length)
    let undoc_count = ($undocumented | length)

    let percentage = if $total == 0 { 0 } else { (($doc_count * 100) / $total) }

    print $"📊 Coverage: ($doc_count)/($total) \(($percentage)%\)\n"

    # Print documented examples
    if ($doc_count > 0) {
        print "✅ Documented Examples:"
        $documented | each { |ex|
            let files = ($ex.files | str join ", ")
            print $"  - ($ex.example) → ($files)"
        }
        print ""
    }

    # Print mentioned examples
    if ($mention_count > 0) {
        print "⚠️  Mentioned but Not Detailed:"
        $mentioned | each { |ex|
            let files = ($ex.files | str join ", ")
            print $"  - ($ex.example) → ($files)"
        }
        print ""
    }

    # Print undocumented examples
    if ($undoc_count > 0) {
        print "❌ Undocumented Examples:"
        $undocumented | each { |ex|
            print $"  - ($ex.example)"
        }
        print ""
    }

    # Coverage goal check
    if ($percentage < 70) {
        print $"⚠️  Warning: Coverage is below 70% goal \(currently ($percentage)%\)"
        exit 0  # Don't fail, just warn
    } else {
        print $"✅ Coverage goal met: ($percentage)% >= 70%"
    }
}

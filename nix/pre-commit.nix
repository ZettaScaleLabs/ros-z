{
  pkgs,
  git-hooks,
  system,
  rustfmtNightly,
  rustToolchain ? pkgs.rust-bin.stable.latest.default,
  docTools ? [
    pkgs.mdbook
    pkgs.mdbook-admonish
    pkgs.mdbook-mermaid
  ],
}:
let
  # yamllint configuration
  yamlFormat = pkgs.formats.yaml { };
  yamllintConfigData = {
    extends = "default";
    rules = {
      line-length = {
        max = 120;
      };
      document-start = "disable";
      truthy = "disable";
    };
  };
  yamllintConfig = yamlFormat.generate ".yamllint.yaml" yamllintConfigData;

  # markdownlint configuration
  markdownlintConfig = {
    # Line length
    "MD013" = false;
    # Multiple consecutive blank lines
    "MD012" = false;
    # Multiple top-level headings in same document
    "MD025" = false;
    # Inline HTML
    "MD033" = {
      allowed_elements = [
        "div"
        "h1"
        "p"
        "strong"
        "a"
        "sub"
        "iframe"
        "script"
        "img"
      ];
    };
    # First line in file should be a top-level heading
    "MD041" = false;
  };
in
git-hooks.lib.${system}.run {
  src = ./..;
  tools = {
    rustfmt = rustfmtNightly;
    cargo = rustToolchain;
    mdbook = pkgs.mdbook;
    mdbook-admonish = pkgs.mdbook-admonish;
    mdbook-mermaid = pkgs.mdbook-mermaid;
  };
  hooks = {
    # Rust tooling
    rustfmt.enable = true;
    taplo.enable = true;

    # Python tooling
    ruff = {
      enable = true;
      description = "Run ruff linter";
      entry = "${pkgs.ruff}/bin/ruff check --fix";
      files = "\\.py$";
      excludes = [ "crates/ros-z-msgs/python/ros_z_msgs_py/types/.*\\.py$" ];
    };

    ruff-format = {
      enable = true;
      description = "Run ruff formatter";
      entry = "${pkgs.ruff}/bin/ruff format";
      files = "\\.py$";
      excludes = [ "crates/ros-z-msgs/python/ros_z_msgs_py/types/.*\\.py$" ];
    };

    mypy = {
      enable = true;
      description = "Run mypy type checker";
      entry = "${pkgs.mypy}/bin/mypy";
      files = "crates/ros-z-py/tests/.*\\.py$|crates/ros-z-py/examples/.*\\.py$";
      pass_filenames = true;
      args = [
        "--ignore-missing-imports"
      ];
    };

    # General tooling
    yamllint = {
      enable = true;
      settings.configPath = "${yamllintConfig}";
    };

    markdownlint = {
      enable = true;
      settings.configuration = markdownlintConfig;
    };

    nixfmt-rfc-style.enable = true;

    # Documentation testing
    mdbook-build = {
      enable = true;
      name = "mdbook-build";
      description = "Build mdbook documentation";
      entry = toString (
        pkgs.writeShellScript "mdbook-build-with-deps" ''
          # Install preprocessors if not already installed
          ${pkgs.mdbook-admonish}/bin/mdbook-admonish install book/ 2>/dev/null || true
          ${pkgs.mdbook-mermaid}/bin/mdbook-mermaid install book/ 2>/dev/null || true
          exec ${pkgs.mdbook}/bin/mdbook build book
        ''
      );
      files = "book/.*\\.(md|toml)$";
      pass_filenames = false;
    };

    mdbook-test = {
      enable = true;
      name = "mdbook-test";
      description = "Test mdbook code examples (rust,no_run blocks via rustdoc --test)";
      entry = toString (
        pkgs.writeShellScript "mdbook-test-with-deps" ''
          # Install preprocessors if not already installed
          ${pkgs.mdbook-admonish}/bin/mdbook-admonish install book/ 2>/dev/null || true
          ${pkgs.mdbook-mermaid}/bin/mdbook-mermaid install book/ 2>/dev/null || true

          # Use rustdoc --test with explicit --extern flags so rust,no_run blocks
          # that import external crates (ros_z, zenoh) compile correctly.
          # Falls back gracefully if build artifacts don't exist yet.
          LIBDIR="target/debug/deps"
          ROS_Z_RLIB=$(ls -t "$LIBDIR"/libros_z-*.rlib 2>/dev/null | head -1)

          if [ -z "$ROS_Z_RLIB" ]; then
            echo "⚠️  libros_z.rlib not found in $LIBDIR — run 'cargo build' first to enable book snippet testing."
            echo "   Skipping book snippet compile check."
            exit 0
          fi

          ZENOH_RLIB=$(ls -t "$LIBDIR"/libzenoh-*.rlib 2>/dev/null | head -1)
          RUSTDOC="${rustToolchain}/bin/rustdoc"

          FAILED=0
          for chapter in book/src/chapters/*.md; do
            ARGS="--test --edition 2021 -L $LIBDIR --extern ros_z=$ROS_Z_RLIB"
            if [ -n "$ZENOH_RLIB" ]; then ARGS="$ARGS --extern zenoh=$ZENOH_RLIB"; fi
            if ! "$RUSTDOC" $ARGS "$chapter"; then
              FAILED=1
            fi
          done
          exit $FAILED
        ''
      );
      files = "book/.*\\.md$|crates/.*/examples/.*\\.rs$";
      pass_filenames = false;
    };
  };
}

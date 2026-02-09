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
      description = "Test mdbook code examples";
      entry = toString (
        pkgs.writeShellScript "mdbook-test-with-deps" ''
          # Install preprocessors if not already installed
          ${pkgs.mdbook-admonish}/bin/mdbook-admonish install book/ 2>/dev/null || true
          ${pkgs.mdbook-mermaid}/bin/mdbook-mermaid install book/ 2>/dev/null || true
          # Run mdbook test with cargo/rustdoc in PATH
          export PATH="${rustToolchain}/bin:$PATH"
          exec ${pkgs.mdbook}/bin/mdbook test book
        ''
      );
      files = "book/.*\\.md$|crates/.*/examples/.*\\.rs$";
      pass_filenames = false;
    };
  };
}

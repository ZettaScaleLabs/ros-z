{
  pkgs,
  git-hooks,
  system,
  rustfmtNightly,
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
    MD013 = false; # Line length
    MD033 = {
      # Inline HTML
      allowed_elements = [
        "div"
        "h1"
        "p"
        "strong"
        "a"
        "sub"
      ];
    };
    MD041 = false; # First line in file should be a top-level heading
  };
in
git-hooks.lib.${system}.run {
  src = ./..;
  tools = {
    rustfmt = rustfmtNightly;
  };
  hooks = {
    # Rust formatter (using nightly for unstable features)
    rustfmt.enable = true;

    # TOML formatter
    taplo.enable = true;

    # YAML linter with relaxed rules
    yamllint = {
      enable = true;
      settings.configPath = "${yamllintConfig}";
    };

    # Markdown linter
    markdownlint = {
      enable = true;
      settings.configuration = markdownlintConfig;
    };

    # Nix formatter
    nixfmt-rfc-style.enable = true;
  };
}

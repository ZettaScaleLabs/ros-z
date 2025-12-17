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
    # Line length
    "MD013" = false;
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
  };
  hooks = {
    rustfmt.enable = true;
    taplo.enable = true;
    yamllint = {
      enable = true;
      settings.configPath = "${yamllintConfig}";
    };
    markdownlint = {
      enable = true;
      settings.configuration = markdownlintConfig;
    };
    nixfmt-rfc-style.enable = true;
  };
}

{
  description = "Robot Development";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let

        pkgs = import nixpkgs {
          inherit system ;
          config = {
            allowUnfree = true;
          };
        };

        libraries = with pkgs; [ gcc.cc.lib ];
        packages = with pkgs; [
          nil
          nixd
          temurin-bin
        ];
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = packages ++ libraries;
          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath libraries;
          PKG_CONFIG_PATH = pkgs.lib.makeSearchPath "lib/pkgconfig" libraries;
        };
      }
    );
}

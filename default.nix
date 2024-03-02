let
    pkgs = import <nixpkgs> { };
in with pkgs;
let
    yosys-cxx17 = pkgs.yosys.overrideAttrs(oa: {
        makeFlags = oa.makeFlags ++ [ "ENABLE_LIBYOSYS=1" "CXXSTD=c++17" ];
        # We actually don't need ENABLE_LIBYOSYS

        doCheck = false;
    });

in
    stdenv.mkDerivation {
        name = "decomp-env";
        buildInputs = [
            pkgs.python311Packages.migen
            pkgs.python311Packages.misoc
            yosys-cxx17
            pkgs.verilator
            pkgs.racket
            pkgs.gtkwave
            pkgs.xdot
            pkgs.readline
        ];
    }

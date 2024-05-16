let
    moz_overlay = import (builtins.fetchTarball https://github.com/mozilla/nixpkgs-mozilla/archive/master.tar.gz);
    pkgs = import <nixpkgs> { overlays = [ moz_overlay ]; };
in with pkgs;
let
    yosys-cxx17 = pkgs.yosys.overrideAttrs(oa: {
        makeFlags = oa.makeFlags ++ [ "ENABLE_LIBYOSYS=1" "CXXSTD=c++17" ];
        # We actually don't need ENABLE_LIBYOSYS

        patches = oa.patches ++ [ ./opt_sdff.patch ];

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
            pkgs.yosys-synlig
            (pkgs.rustChannelOf { date = "2022-09-27"; channel = "nightly"; }).rust
        ];
    }

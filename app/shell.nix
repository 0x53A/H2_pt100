{ pkgs ? import <nixpkgs> {} }:

let
  libPath = with pkgs; lib.makeLibraryPath [
    libGL
    libxkbcommon
    wayland
  ];
in
pkgs.mkShell {
  buildInputs = with pkgs; [
    pkg-config
    openssl
    udev
    #cargo
    #rustc
    #rust-analyzer
  ];
  
  LD_LIBRARY_PATH = libPath;
}

local $/; open T,"<template.html"; $t=<T>; open D,"<data.json"; $d=<D>; $d=~s#</#<\/#g; $t=~s#/\*DATA\*/#$d#;
open O,">joint-stress-313.html"; print O $t;
open P,">preview.html"; print P "<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><style>body{margin:0}</style></head><body>$t</body></html>";

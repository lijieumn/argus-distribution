#!/bin/bash

# ensure predictable working directory
cd "$(dirname "$0")"

for name in box_and_cone_0_{00,17,34,50};
do
    outdir=../output/figures/fig_11/$name
    mkdir -p $outdir
    ../build/apps/argus-cloth simulateoffline ../conf/$name.json $outdir
done

pdflatex fig_11.tex

#!/bin/bash

tar xfz data_private_license.tgz
pushd data_private_license/meshes/character/
for name in *.tgz; do
echo "Extracting $name..."
tar xfz $name
done
popd
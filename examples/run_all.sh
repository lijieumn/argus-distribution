echo "WARNING: Please be aware that running all the examples can take a few days (each character animation can take 5 to more than 10 hours, also depending on your computer specifications)."

python examples/belt_high_tension.py -s 1 -d 0
python examples/box_and_cone.py -s 1 -d 0
python examples/cards_high_friction.py -s 1 -d 0
python examples/cards_low_friction.py -s 1 -d 0
python examples/drag_cloth.py -s 1 -d 0

# Following are character examples
tar xfz argus_data_private_license.tgz
pushd data_private_license/meshes/character/
tar xfz *.tgz
popd
python examples/character/Arabesque-0-3.py -s 1 -d 0

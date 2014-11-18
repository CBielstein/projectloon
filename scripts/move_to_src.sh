# Cameron Bielstein 11/18/2014
# move_to_src.sh
# I wrote a previous script that didn't understand ns3 src folder correctly.
# This script fixes it by moving this in to src.

# Change to NS3DIR
cd $NS3DIR

# Remove ProjectLoon folder
rm -rf ProjectLoon

# Revert master wscripts file
rm wscript
cp /ns3/bake/source/ns-3.21/wscript ./

# Put ProjectLoon repo in to src
cd src
git clone https://github.com/CBielstein/ProjectLoon.git

cd -
cd -

echo "Go change your goloon alias in ~/.profile to point to ...bake/ns-3.21/src/ProjectLoon instead of ...bake/ns-3.21/ProjectLoon"

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
git clone https://github.com/CBielstein/projectloon.git
cd -

# Update waf configuration
./waf --build-profile=debug --enable-examples --enable-tests configure

# Update waf build
./waf

# Run test hello-loon
./waf --run hello-loon

cd -

echo "Your goloon alias is now out of date."
echo "Edit your profile (vim ~/.profile) and change alias goloon='cd \$NS3DIR/ProjectLoon' to alias goloon='cd \$NS3DIR/src/projectloon'"

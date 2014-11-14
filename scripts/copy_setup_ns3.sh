# !/bin/bash

# Cameron Bielstein, 11/14/2014
# This is the script that copies and set up ns3
# This is extremely specific to our situation, as it assumes ns3 is already installed in /ns3/bake
# It also references our github repo specifically
# It's only added to the repo for our tracking purposes
# Maybe this should be modified to get ns3 from bake and build? That takes way longer than the copy, though

# End result will be
#    * $PWD/bake directory that stores all the necessary stuff
#    * Checked out ProjectLoon directory in ns3 directory
#    * Updated wscript file to include ProjectLoon in waf builds
#    * Added waff command to build with waf from any directory and have outputs to the proper place

# Here we goooo

# copy ns3 to current 
echo "Copy ns3 to $PWD"
cp -r /ns3/bake $PWD/bake

# configure waf and let it do its thing
echo "Reconfigure waf for some reason"
cd bake/source/ns-3.12
./waf configure --enable-examples --enable-tests
./waf

# check out the github repo
echo "Clone the ProjectLoon repo"
git clone https://github.com/CBielstein/ProjectLoon.git

# replace wscript
echo "Update the wscript"
rm wscript
cp ProjectLoon/scripts/wscript ./

# modifying .profile to add handy commands, hope you're cool with this
# waff and waffc are for your own convenience. Makefiles should referece waf more directly. Probably with waf --cwd or maybe $NS3DIR
echo "Append to .profile"
echo "Commands added: waff, waffc, goloon"
echo "  waff: runs waf from wherever you are."
echo "  waffc: runs waf from wherever you are and sends output to your current folder. I think this is desirable, but not sure."
echo "  goloon: alias for a cd to the ProjectLoon folder"

echo "" >> ~/.profile
echo "# Added by Cameron's script for ns3 and ProjectLoon work" >> ~/.profile
echo "export NS3DIR=\"$PWD\"" >> ~/.profile
echo "alias goloon=\"cd $PWD/ProjectLoon\"" >> ~/.profile
echo "function waff { cd $NS3DIR && ./waf $* ; }"
echo "function waffc {" >> ~/.profile
echo "    CWD=\"$PWD\"" >> ~/.profile
echo "    cd $NS3DIR >/dev/null" >> ~/.profile
echo "   ./waf --cwd=\"$CWD\" $*" >> ~/.profile
echo "    cd - >/dev/null" >> ~/.profile
echo "  }" >> ~/.profile

rm -rf ~/.openrave
mkdir ~/.openrave
touch ~/.openrave/youbot_traj.xml

cp -r or_utils/collision1/* ~/.openrave/.
cp -r or_utils/ik5d/* ~/.openrave/.
cp -r or_utils/ik6d/* ~/.openrave/.
cp -r or_utils/inverse_reachability/* ~/.openrave/.
cp -r or_utils/linkstatistics/* ~/.openrave/.
cp -r or_utils/workspace/* ~/.openrave/.  


#!/bin/bash
for i in *iv; do
j=$(echo $i|sed -e "s/iv/wrl/");
sed -e "s/^#Inventor/#VRML/" $i > $j ;
echo "$i -> $j" ;
done

#!/bin/bash

#
# Script for changing properly the name of the template in an easy way.
#
# Author: Santiago Iregui (santiago.iregui@kuleuven.be)

export RED='\e[38;5;198m'
export DEFAULTC='\e[39m'
CWD=$(pwd)

echo -e "\v \e[31m${bold}Warning: $DEFAULTC ${normal}"
read -p "You are about to change the name of the package. It is recommended that you delete the local git repository first (rm -rf <package_directory>/.git). Are you sure you want to proceed? (y/n) " -n 1 -r
echo " "

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
  echo -e 'The name of the package was NOT changed'
  return
fi

echo -e "\vPlease provide the location of the package. Press ENTER if OK, otherwise edit: "
# retrieve the directory in which this script is located:
package_directory="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
# ask for user input + suggest retrieved directory as default value:
read -e -i "$package_directory" -p "" BASE

# if no input, then take the default value:
BASE="${BASE:-$package_directory}"
package_name=${BASE##*/}

echo -e "\v \e[31m${bold}Warning: $DEFAULTC ${normal} Please make sure that the following is the name of the package:"
echo -e "\v \t $package_name \v"
echo -e "If not, please abort and provide the proper package directory in the previous step.\v"

read -p "Please provide the new name of the package or type q to quit: " NEW_NAME

if [ "x$NEW_NAME" = "x" ]; then
  echo -e 'You need to provide a non empty name. Operation canceled'
  return
fi
if [ "$NEW_NAME" = "q" ]; then
  echo -e 'You have canceled the operation. The name of the package was NOT changed'
  return
fi

echo "The new name is: $NEW_NAME"
cd $package_directory
# This changes the name recursively in every file contained in the package:
egrep -lRZ "$package_name" . | xargs -0 -l sed -i -e "s/$package_name/$NEW_NAME/g"

# Now change the name of the package folder:
cd ..
mv $package_name $NEW_NAME

#Returns to the initial directory of the terminal
if [ "${CWD##*/}" == "$package_name" ]; then
  cd $NEW_NAME
else
  cd $CWD
fi

echo -e "\v \e[32m The package was configured correctly.$DEFAULTC"
echo -e "It is recommended that now you initialize a new local git repository by running:"
echo -e "\v\tgit init"

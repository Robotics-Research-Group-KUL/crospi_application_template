#!/bin/bash
#
# Script for changing properly the name of the template in an easy way.
#
# Author: Santiago Iregui (santiago.iregui@kuleuven.be)

bold=$(tput bold)
normal=$(tput sgr0)
RED='\e[38;5;198m'
DEFAULTC='\e[39m'
CWD=$(pwd)

echo -e "\v ${RED}${bold}Warning:${DEFAULTC}${normal}"
read -p "You are about to change the name of the package. It is recommended that you delete the local git repository first (rm -rf <package_directory>/.git). Are you sure you want to proceed? (y/n) " -n 1 -r
echo " "

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
  echo -e 'The name of the package was NOT changed'
  exit 0
fi

echo -e "\vPlease provide the location of the package. Press ENTER if OK, otherwise edit: "
package_directory="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
read -e -i "$package_directory" -p "" BASE
BASE="${BASE:-$package_directory}"
package_name=${BASE##*/}

echo -e "\v ${RED}${bold}Warning:${DEFAULTC}${normal} Please make sure that the following is the name of the package:"
echo -e "\v\t $package_name \v"
echo -e "If not, please abort and provide the proper package directory in the previous step.\v"

read -p "Please provide the new name of the package or type q to quit: " NEW_NAME

if [ -z "$NEW_NAME" ]; then
  echo -e 'You need to provide a non-empty name. Operation canceled.'
  exit 1
fi
if [ "$NEW_NAME" = "q" ]; then
  echo -e 'You have canceled the operation. The name of the package was NOT changed.'
  exit 0
fi

echo "The new name is: $NEW_NAME"
cd "$package_directory" || exit 1

# Recursively replace occurrences, excluding .git and build dirs
grep -lRZ "$package_name" . --exclude-dir=.git --exclude-dir=build --exclude-dir=install | \
xargs -0 -l sed -i -e "s|$package_name|$NEW_NAME|g"

# Rename the package folder
cd ..
mv "$package_name" "$NEW_NAME"

# Return to the original directory
if [ "${CWD##*/}" == "$package_name" ]; then
  cd "$NEW_NAME" || exit 1
else
  cd "$CWD" || exit 1
fi

echo -e "\v \e[32mThe package was configured correctly.${DEFAULTC}"
echo -e "It is recommended that you now initialize a new local git repository by running:"
echo -e "\v\tgit init"

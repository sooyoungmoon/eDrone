#! /bin/bash


ROOT=$PWD

cd build

for PROJECT in `find $PWD -name .project`; do

    DIR=`dirname $PROJECT`

    echo $DIR

    cd $DIR

    awk -f $(rospack find mk)/eclipse.awk .project > .project_with_env && mv .project_with_env .project

done

cd $ROOT

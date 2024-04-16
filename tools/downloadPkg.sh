#!/usr/bin/env bash

function downloadPkg() {
user='guest'
password='Hozon2022'
url='http://10.6.133.99:8082/artifactory'

depend='./depend'


ENABLE_SINGLE_COMPILE_PROTO=$2
echo "ENABLE_SINGLE_COMPILE_PROTO : $ENABLE_SINGLE_COMPILE_PROTO"

if [ ! -d ~/.jfrog ]; then
    mkdir -p ~/.jfrog && cp ./tools/jfrog/jfrog-cli.conf.v5 ~/.jfrog/
fi

# package alias in version.json
# pkgAlias=(ADF NOS third_party CUDA)
pkgAlias=(ADF middleWare third_party CUDA)
pkgDependRelDir=(ap-release nos third_party third_party/x86/cuda)
pkgJfrogRelDir=(ADC/EP40/ADF/release ORIN/EP41/NOS/release EP40_MDC_TP/build nvidia/cuda )

for i in $(seq 0 `expr ${#pkgAlias[@]} - 1`); do
    pkg=${pkgAlias[i]}
    dependRelDir=${pkgDependRelDir[i]}
    jfrogRelDir=${pkgJfrogRelDir[i]}
    echo "============== okok dependRelDir: ${dependRelDir}"
    currPkg=$(python3 -c "import sys, json;
with open('version.json','r') as f: print(json.load(f)['ORIN']['EP41']['${pkg}']['name'])")

    verInfo=''
    # if [ "$pkg" == "NOS" ]; then
    #   if [ "$1" == "x86" ]; then
    #     nos_platform=x86_2004
    #     nos_key=x86
    #   elif [ "$1" == "orin" ]; then
    #     nos_platform=orin
    #     nos_key=ORIN
    #   elif [ "$1" == "mdc" ]; then
    #     nos_platform=x86_2004
    #     nos_key=x86
    #   fi
    # else
      if [ ${dependRelDir} == 'nos' ]; then
        if [ -f $depend/${dependRelDir}/nos_orin/version.json ]; then
          verInfo=$(python3 -c "import sys, json;
with open('$depend/${dependRelDir}/nos_orin/version.json','r') as f: print(json.load(f)['ORIN']['EP41']['releaseVersion'])")
        fi
      fi

      if [ -f $depend/${dependRelDir}/version.json ]; then
        if [ ${dependRelDir} == 'third_party' ]; then
          verInfo=$(python3 -c "import sys, json;
with open('$depend/${dependRelDir}/version.json','r') as f: print(json.load(f)['releaseVersion'])")
        elif [ ${dependRelDir} == 'third_party/x86/cuda' ]; then
          verInfo=$(python3 -c "import sys, json;
with open('$depend/${dependRelDir}/version.json','r') as f: print(json.load(f)['releaseVersion'])")
        elif [ ${dependRelDir} == 'ap-release' ]; then
          verInfo=$(python3 -c "import sys, json;
with open('$depend/${dependRelDir}/version.json','r') as f: print(json.load(f)['ADC']['EP40']['releaseVersion'])")
        else
          verInfo=$(python3 -c "import sys, json;
with open('$depend/${dependRelDir}/version.json','r') as f: print(json.load(f)['ORIN']['EP41']['releaseVersion'])")
        fi
      fi
    # fi

    if [[ "$verInfo" != "$currPkg" ]]; then
        echo "[INFO] Update ${pkg}: ${verInfo} ==> ${currPkg}"
        ./tools/jfrog/jfrog rt dl --flat --user=$user --password=$password --url=$url ${jfrogRelDir}/$currPkg*
        echo "[INFO] tar -xf $currPkg.tar.gz "
        # special for third_party and CUDA
        if [[ "$pkg" == "third_party" ]]; then
          dest=$depend
          rm -rf $depend/${dependRelDir}
        elif [[ "$pkg" == "CUDA" ]]; then
          dest=$depend/third_party/x86
          mkdir -p $dest
          rm -rf $depend/${dependRelDir}

          dest_2004=$depend/third_party/x86_2004
          if [[ -d $dest_2004 ]]; then
            rm -rf $dest_2004/cuda
            tar -xf $currPkg.tar.gz -C $dest_2004
          fi
        else
          dest=$depend/${dependRelDir}
          mkdir -p $dest
          rm -rf $dest/*
        fi
        tar -xf $currPkg.tar.gz -C $dest && rm -f $currPkg.tar.gz
    fi
done

thirdPartyPath='./depend'
if [ -f $thirdPartyPath/nos/orin/version.json ]; then
    ProtoVersion=$(python3 -c "import sys, json;
with open('$thirdPartyPath/nos/orin/version.json','r') as f: print(json.load(f)['ORIN']['EP41']['proto'])")
fi

if [ "$ENABLE_SINGLE_COMPILE_PROTO" = "OFF" ]; then
    pushd $thirdPartyPath/proto
    git fetch --all
    git reset ${ProtoVersion} --hard
    echo "Submodule path 'proto': checked out :" ${ProtoVersion}
    popd
fi
}

downloadPkg "$@"

#!/usr/bin/env bash

function downloadPkg() {
user='guest'
password='Hozon2022'
url='http://10.6.133.99:8082/artifactory'

depend='./depend'

if [ ! -d ~/.jfrog ]; then
    mkdir -p ~/.jfrog && cp ./tools/jfrog/jfrog-cli.conf.v5 ~/.jfrog/
fi

# package alias in version.json
# pkgAlias=(ADF NOS third_party CUDA)
pkgAlias=(ADF NOS third_party CUDA PCPBASE PCPCOMBOARD PCPLIB)
# pkgDependRelDir=(ap-release nos third_party third_party/x86/cuda)
pkgDependRelDir=(ap-release nos third_party third_party/x86/cuda perception-base/release perception-common-onboard/release perception-lib/release)
# pkgJfrogRelDir=(EP40_MDC_ADF/release ORIN/EP41/NOS/release EP40_MDC_TP/build nvidia/cuda)
pkgJfrogRelDir=(EP40_MDC_ADF/release ORIN/EP41/NOS/release EP40_MDC_TP/build nvidia/cuda EP40_MDC_PCP/BASE/release EP40_MDC_PCP/COMBOARD/release EP40_MDC_PCP/LIB/release)

for i in $(seq 0 `expr ${#pkgAlias[@]} - 1`); do
    pkg=${pkgAlias[i]}
    dependRelDir=${pkgDependRelDir[i]}
    jfrogRelDir=${pkgJfrogRelDir[i]}
    
    currPkg=$(python3 -c "import sys, json;
with open('version.json','r') as f: print(json.load(f)['${pkg}'])")

    verInfo=''
    if [ "$pkg" == "NOS" ]; then
      if [ "$1" == "x86" ]; then
        nos_platform=x86_2004
        nos_key=x86
      elif [ "$1" == "orin" ]; then
        nos_platform=orin
        nos_key=ORIN
      elif [ "$1" == "mdc" ]; then
        nos_platform=x86_2004
        nos_key=x86
      fi
      versionfile=$depend/${dependRelDir}/nos_${nos_platform}/version.json
      if [ -f $versionfile ]; then
        verInfo=$(python3 -c "import sys, json;
with open('$versionfile','r') as f: print(json.load(f)['ORIN']['EP41']['middleWare'])")
      fi
    else
      if [ -f $depend/${dependRelDir}/version.json ]; then
        verInfo=$(python3 -c "import sys, json;
with open('$depend/${dependRelDir}/version.json','r') as f: print(json.load(f)['releaseVersion'])")
      fi
    fi

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

# currThirdParty=$(python3 -c "import sys, json;
# with open('version.json','r') as f: print(json.load(f)['third_party'])")

# echo "current third_party in version.json is $currThirdParty"
# TPVersionInfo=''
# if [ -f $depend/third_party/version.json ]; then
#     TPVersionInfo=$(python3 -c "import sys, json;
# with open('$depend/third_party/version.json','r') as f: print(json.load(f)['releaseVersion'])")
# fi

# if [[ "$TPVersionInfo" != "$currThirdParty" ]]; then
#     echo "current third_party in third_party/version.json is $TPVersionInfo"
#     ./tools/jfrog/jfrog rt dl --flat --user=$user --password=$password --url=$url EP40_MDC_TP/build/$currThirdParty*
#     echo "tar -xf $currThirdParty.tar.gz "
#     rm -rf $depend/third_party && tar -xf $currThirdParty.tar.gz -C $depend && rm -f $currThirdParty.tar.gz
# fi

# currNOS=$(python3 -c "import sys, json;
# with open('version.json','r') as f: print(json.load(f)['NOS'])")
# echo "current ADF in version.json is $currNOS"
# NOSVersionInfo=''
# if [ -f $depend/nos/version.json ]; then
#     NOSVersionInfo=$(python3 -c "import sys, json;
# with open('$depend/nos/version.json','r') as f: print(json.load(f)['releaseVersion'])")
# fi

# if [[ "$NOSVersionInfo" != "$currNOS" ]]; then
#     echo "current nos in nos/version.json is $NOSVersionInfo"
#     ./tools/jfrog/jfrog rt dl --flat --user=$user --password=$password --url=$url ORIN_NOS/nos/release/$currNOS*
#     echo "tar -xf $currNOS.tar.gz "
#     mkdir -p $depend/nos && rm -rf $depend/nos/* && tar -xf $currNOS.tar.gz -C $depend/nos && rm -f $currNOS.tar.gz
# fi

# currPCPBASE=$(python3 -c "import sys, json;
# with open('version.json','r') as f: print(json.load(f)['PCPBASE'])")

# echo "current PCPBASE in version.json is $currPCPBASE"
# PCPBASEVersionInfo=''
# if [ -f $depend/perception-base/release/version.json ]; then
#     PCPBASEVersionInfo=$(python3 -c "import sys, json;
# with open('$depend/perception-base/release/version.json','r') as f: print(json.load(f)['releaseVersion'])")
# fi
# echo "current PCPBASEVersionInfo in version.json is $PCPBASEVersionInfo"

# if [[ "$PCPBASEVersionInfo" != "$currPCPBASE" ]]; then
#     echo "current cuda in cuda/version.json is $PCPBASEVersionInfo"
#     ./tools/jfrog/jfrog rt dl --flat --user=$user --password=$password --url=$url EP40_MDC_PCP/BASE/release/$currPCPBASE*
#     echo "tar -xf $currPCPBASE.tar.gz "
#     mkdir -p $depend/perception-base/release && rm -rf $depend/perception-base/release/* && tar -xf $currPCPBASE.tar.gz -C $depend/perception-base/release/ && rm -f $currPCPBASE.tar.gz
# fi
}

downloadPkg "$@"

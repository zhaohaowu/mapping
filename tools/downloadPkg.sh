#!/usr/bin/env bash

function downloadPkg() {
user='guest'
password='Hozon2022'
url='http://10.6.133.99:8082/artifactory'

thirdPartyPath='./third_party'

if [ ! -d ~/.jfrog ]; then
    mkdir -p ~/.jfrog && cp ./tools/jfrog/jfrog-cli.conf.v5 ~/.jfrog/
fi

currThirdParty=$(python3 -c "import sys, json;
with open('version.json','r') as f: print(json.load(f)['third_party'])")

echo "current third_party in version.json is $currThirdParty"
TPVersionInfo=''
if [ -f $thirdPartyPath/third_party/version.json ]; then
    TPVersionInfo=$(python3 -c "import sys, json;
with open('$thirdPartyPath/third_party/version.json','r') as f: print(json.load(f)['releaseVersion'])")
fi

if [[ "$TPVersionInfo" != "$currThirdParty" ]]; then
    echo "current third_party in third_party/version.json is $TPVersionInfo"
    ./tools/jfrog/jfrog rt dl --flat --user=$user --password=$password --url=$url EP40_MDC_TP/build/$currThirdParty*
    echo "tar -xf $currThirdParty.tar.gz "
    rm -rf $thirdPartyPath/third_party && tar -xf $currThirdParty.tar.gz -C $thirdPartyPath && rm -f $currThirdParty.tar.gz
fi

currNOS=$(python3 -c "import sys, json;
with open('version.json','r') as f: print(json.load(f)['NOS'])")
echo "current ADF in version.json is $currNOS"
NOSVersionInfo=''
if [ -f $thirdPartyPath/nos/version.json ]; then
    NOSVersionInfo=$(python3 -c "import sys, json;
with open('$thirdPartyPath/nos/version.json','r') as f: print(json.load(f)['releaseVersion'])")
fi

if [[ "$NOSVersionInfo" != "$currNOS" ]]; then
    echo "current nos in nos/version.json is $NOSVersionInfo"
    ./tools/jfrog/jfrog rt dl --flat --user=$user --password=$password --url=$url ORIN_NOS/nos/release/$currNOS*
    echo "tar -xf $currNOS.tar.gz "
    mkdir -p $thirdPartyPath/nos && rm -rf $thirdPartyPath/nos/* && tar -xf $currNOS.tar.gz -C $thirdPartyPath/nos && rm -f $currNOS.tar.gz
fi

currPCPBASE=$(python3 -c "import sys, json;
with open('version.json','r') as f: print(json.load(f)['PCPBASE'])")

echo "current PCPBASE in version.json is $currPCPBASE"
PCPBASEVersionInfo=''
if [ -f $thirdPartyPath/perception-base/release/version.json ]; then
    PCPBASEVersionInfo=$(python3 -c "import sys, json;
with open('$thirdPartyPath/perception-base/release/version.json','r') as f: print(json.load(f)['releaseVersion'])")
fi
echo "current PCPBASEVersionInfo in version.json is $PCPBASEVersionInfo"

if [[ "$PCPBASEVersionInfo" != "$currPCPBASE" ]]; then
    echo "current cuda in cuda/version.json is $PCPBASEVersionInfo"
    ./tools/jfrog/jfrog rt dl --flat --user=$user --password=$password --url=$url EP40_MDC_PCP/BASE/release/$currPCPBASE*
    echo "tar -xf $currPCPBASE.tar.gz "
    mkdir -p $thirdPartyPath/perception-base/release && rm -rf $thirdPartyPath/perception-base/release/* && tar -xf $currPCPBASE.tar.gz -C $thirdPartyPath/perception-base/release/ && rm -f $currPCPBASE.tar.gz
fi
}

downloadPkg "$@"

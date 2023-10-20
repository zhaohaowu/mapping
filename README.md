## 项目名称
NCP建图定位代码仓库

## 代码书写要求
> 列出代码书写的相关要求
* [代码规范](https://hozonauto.feishu.cn/wiki/PzUWwj5U6iikt4krZxFcSvPZnUe)
* [Mapping仓库基础开发规范对齐](https://hozonauto.feishu.cn/wiki/S3pswYCV0i4uM1kZwOKcuuDPnCe)


## 编译方式
* 查看编译选项
bash build.sh -h
* 基于orin平台和adf-lite进行编译
bash build.sh orin --clean
* 基于x86和adf-lite进行编译
bash build.sh x86 --clean
* 基于x86和cyber进行编译
bash build.sh x86 --cyber --clean
* 基于adc和adf进行编译
bash build.sh mdc --clean
* 编译参数概览
  --ind: 自身或台架测试时，将依赖的三方库及其它模块产出打包至release/mal_$platform/lib下，编译实车产出时勿加


## 代码开发流程
> 开发/合入/评审/发布
* 新的迭代开始，开发人员从master拉取个人分支，命名规范fea_xxx/dev_xxx，开发完成后合入master
* master分支合并后，自动CICD到开发环境
* 从master拉取预发布分支release，将该分支部署到测试环境进行测试
* 存在bug，通过从release拉取hotfix分支进行修复，命名规范fix_xxx，完成后合入release，并且删除fix分支
* 待release稳定后，反向合并到maste

## 技术架构
> 使用的技术框架或系统架构图等相关说明，请填写在这里

##编译产出目录结构
> 产出结构与系统部协定如下

相关MR
> https://qingluan.hozonauto.com/geelib/repoMergeDetail?repoId=algorithm/mapping&mergeId=127

运行cyber时
```
cd workspace/release/mal_x86/scripts && source set_envs.bash
cd workspace/release/mal_x86/runtime_service/cyber && source setup.bash
cyber_launch start launch/mapping_all.launch
```


```
release
├── mal_mdc
│   ├── conf  // 外部conf，最终拷贝到车端/opt/app/1/conf/
│   ├── data
│   ├── lib   // 外部公用的lib, 目前包含third_party、europa_[commonn|map], 定位目前不产生公共库, 不加--ind为空
│   ├── runtime_service
│   │   └── mapping
│   │       ├── bin
│   │       ├── conf
│   │       │   └── mapping
│   │       ├── etc
│   │       └── lib
│   └── scripts
|       └── mal_start.sh  // adc平台运行脚本，来自workspace/onboard/onboard_adc/production/scripts
├── mal_orin
│   ├── conf  // 外部conf，最终拷贝到车端/app/conf/
│   ├── data
│   ├── lib   // 依赖库，目录会拷贝至车端/app/lib
│   ├── runtime_service
│   │   └── mapping
│   │       ├── bin  // 包含hz_mapping, 由adf-lite-process重命名而来
│   │       ├── conf
│   │       │   ├── lite     // adf-lite相关的配置文件
│   │       │   └── mapping  // 定位建图业务自身的配置文件
│   │       ├── etc
│   │       └── lib  // 定位建图自身产生的so
│   └── scripts
|       └── mal_start.sh  // orin平台adf-lite运行脚本，来自workspace/onboard/onboard_lite/production/scripts
└── mal_x86
    ├── conf
    ├── data
    ├── lib
    ├── runtime_service
    │   └── cyber  // --cyber时是mapping的超链接
    │   └── mapping
    │       ├── bin
    │       ├── conf
    │       │   ├── lite
    │       │   └── mapping
    │       ├── etc
    │       └── lib
    ├── scripts
    |   ├── mal_start.sh   // x86平台adf-lite运行脚本，来自workspace/onboard/onboard_lite/production/scripts  
    |   └── set_envs.bash  // --cyber编译由于cyber本身运行目录比较固定, cyber相关的so仍处于内存目录
    |                      // 但是cyber依赖的部分三方库位于外层lib目录下，需运行此脚本加载库目录
    └── test
```

## FAQ
> 列出常见问题
* ~~基于x86和adf-lite编译，目前需要map仓库切换到dev_hozon_n,common切换到dev_hozon_n,proto仓库切换到dev_hozon_nn.~~
* adf-lite中间件下，ADFLITE_ROOT_PATH环境变量比较关键，adf-lite的配置文件中也会使用。
  目前 --flagfile 无法通过adf-lite生效，定位onboard代码中会读取此变量拼接配置文件


## Contact us

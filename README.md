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


## 代码开发流程
> 开发/合入/评审/发布 
* 新的迭代开始，开发人员从master拉取个人分支，命名规范fea_xxx/dev_xxx，开发完成后合入master
* master分支合并后，自动CICD到开发环境
* 从master拉取预发布分支release，将该分支部署到测试环境进行测试
* 存在bug，通过从release拉取hotfix分支进行修复，命名规范fix_xxx，完成后合入release，并且删除fix分支
* 待release稳定后，反向合并到maste

## 技术架构
> 使用的技术框架或系统架构图等相关说明，请填写在这里  

## FAQ
> 列出常见问题 
* 基于x86和adf-lite编译，目前需要map仓库切换到dev_hozon_n,common切换到dev_hozon_n,proto仓库切换到dev_hozon_nn.


## Contact us

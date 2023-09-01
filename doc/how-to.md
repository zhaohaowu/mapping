## 如何编译

当前仅支持`x86`编译。

```shell
bash build.sh x86
```

## 启动cyber

```shell
cd release/cyber
source setup.bash
cyber_launch start launch/mapping_all.launch
```

上面命令以`mapping_all.launch`为例，也可以指定其它新增的launch文件。

## 一些规范

### 目录结构

遵循：
```
mapping
  |-- build     # 编译产生的临时文件
  |-- cmake
  |-- conf
    |-- mapping # 配置文件放到对应子模块目录里
      |-- dr
      |-- local_mapping
      |-- location
      |-- map_fusion
  |-- dag       # cyber的dag文件
  |-- depend    # 所有依赖的外部仓库或外部包
  |-- doc       # 文档
  |-- interface # 内外部数据接口
  |-- launch    # cyber的launch文件
  |-- modules   # 核心业务代码
    |-- dr
    |-- local_mapping
    |-- location
    |-- map_fusion
  |-- release   # 产出目录
  |-- test      # 测试用例, 各子模块单独放
    |-- dr
    |-- local_mapping
    |-- location
    |-- map_fusion
  |-- tools # 工具、脚本
  |-- build.sh # 统一的编译脚本
```

### 统一库的编译命令

为了方便给库加上统一的前缀，使用自定义的cmake宏`add_mapping_library`（而非默认的`add_library`）来编译库，比如：
```cmake
set(MAP_FUSION_SRCS
    src/map_fusion.cc
)

add_mapping_library(mf ${MAP_FUSION_SRCS})
```

这样编出的库`mf`会自动加上统一的前缀，当前默认是`mapping_`，即实际`mf`库的名字是`libmapping_mf.so`。

### cyber相关

编译脚本会将最后的产出打包在`release/x86`，其中包含所有生成的库，以及配置文件、dag文件、launch文件等，以及完整的cyber环境。

并且为了能正常运行cyber，编译脚本里会自动创建一个软连接`release/cyber`指向`release/x86`。
运行cyber时需要进到`release/cyber`里source环境变量：

```shell
cd release/cyber
source setup.bash
```

cyber的`CYBER_PATH`就是`release/cyber`，因此在dag文件及launch文件里写文件路径时都使用`release/cyber`下的相对路径，比如`launch/mapping_all.launch`里：

```
<cyber>
  <module>
    <name>map_fusion</name>
    <dag_conf>dag/mapping_map_fusion.dag</dag_conf>
    <process_name>map_fusion</process_name>
  </module>
</cyber>
```

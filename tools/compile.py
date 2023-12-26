#!/usr/bin/env python3
import argparse
import datetime
import os
import os.path as osp
import subprocess as sp
import shutil

from multiprocessing import cpu_count

PKG_ALIAS = ['base', 'comboard', 'lib']
PKG_DIRS = ['perception-base', 'perception-common-onboard', 'perception-lib']
PKG_LIBS = ['perception-base', 'perception-onboard-common', 'perception-lib']
PKG_CMAKE_ENABLES = ['-DMAPPING_ENABLE_COMPILE_BASE', '-DMAPPING_ENABLE_COMPILE_COMBOARD', '-DMAPPING_ENABLE_COMPILE_LIB']

def parse_args():
    p = argparse.ArgumentParser(description='compiling usage and options',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    # 位置参数
    p.add_argument('platform', choices=['mdc', 'orin', 'x86', 'all'], help='choose platform')
    # 开关类型
    p.add_argument('--verbose', action='store_true', help='enable verbose mode for cmake')
    p.add_argument('--release', action='store_true', help='set CMAKE_BUILD_TYPE=Release, default Debug')
    p.add_argument('--ut', action='store_true', help='use unit test or not, default false')
    p.add_argument('--clean', action='store_true', help='remove cmake cache first, then build again')
    p.add_argument('--gcc', action='store_true', help='build lib with gcc,default llvm')
    p.add_argument('--make', action='store_true', help='directly execute make, will not compile again')
    p.add_argument('--base', action='store_true', help='enable active compile submodule perception-base')
    p.add_argument('--comboard', action='store_true', help='enable active compile submodule perception-common-board')
    p.add_argument('--lib', action='store_true', help='enable active compile submodule perception-lib')
    p.add_argument('--prefix', default='mapping_', help='prefix for all built libraries')
    p.add_argument('--rviz', action='store_true', help='enable building targets for using rviz')
    p.add_argument('--tool', action='store_true', help='enable building Mapping_tools, should be used with --cyber')
    p.add_argument('--cyber', action='store_true', help='enable cyber')
    p.add_argument('--ind', action='store_true', help='copy third party libs')
    p.add_argument('--plugin', action='store_true', help='build with mal_plugin')
    # 默认值类型
    p.add_argument('--workspace', default=None, help='root of code repository')
    p.add_argument('-j', default=6, dest="jobs", type=int, help='make -j')
    p.add_argument('--hdmap', default='/usr/local/hd_map', help='hd_map path')

    return p.parse_args()

# 执行shell命令
def execute_shell(cmd,shell=True,stdout=None):
    result = sp.run(cmd,shell=shell,stdout=stdout)
    if (result.returncode != 0) :
        raise(f"{result.returncode}, {cmd} failed")
    return result.stdout

def LOG_INFO(*msg):
    print('\033[36m[INFO]', *msg)

def LOG_ERROR(*msg):
    print('\033[31m[ERROR]', *msg)

def set_env(var, value):
    if var in ['LD_LIBRARY_PATH', 'PATH']:
        if os.environ.get(var) is not None:
            os.environ[var] = value + ":" + os.environ[var]
        else:
            os.environ[var] = value
    else:
        os.environ[var] = value

def copy_file(source_path, destination_path):
    # 检查目标路径是否存在，不存在则创建
    if not os.path.exists(destination_path):
        os.makedirs(destination_path)
    # 复制文件
    shutil.copy2(source_path, destination_path)

def del_remain_external_lib(workspace, platform, release_directory, **kwargs):
    """delete residual third lib with no --ind params"""
    if kwargs['ind']:
        return

    prefix = osp.join(workspace, release_directory, "mal_" + platform, 'lib')
    tgt_names = ['libglobalproto.so']
    tgt_names = [osp.join(prefix, n) for n in tgt_names]

    cmd = 'rm {}'.format(' '.join(tgt_names))
    LOG_INFO(cmd)
    execute_shell(cmd)

def cmake_build(workspace, platform, build_directory, cmake_args, jobs, verbose=False):
    """执行cmake 编译过程, 结束后切换 current working to workspace
    workspace: 代码仓根路径
    build_directory: cmake 编译生成目录， 不是编译后的产出目录
    cmake_args: cmake 编译选项设置  -DCMAKE_BUILD_TYPE=Release ..
    jobs: 指定编译使用的cpu数目 make -j8
    verbose: 打开make时的详细信息
    """

    """子模块修改后的编译"""
    for (pkg, pkg_dir) in zip(PKG_ALIAS, PKG_DIRS):
        if kwargs[pkg]:
            sp.run('rm -rf {}'.format(workspace+"/depend/"+pkg_dir+"/release"), shell=1)
            copy_header_files_by_path(workspace+"/depend/"+pkg_dir, workspace+"/depend/"+pkg_dir+"/release/"+platform+"/include/"+pkg_dir)
            copy_file(workspace+"/depend/"+pkg_dir+"/version.json", workspace+"/depend/"+pkg_dir+"/release/")

    date_s = datetime.datetime.now()
    # cmake ..
    os.makedirs(build_directory, exist_ok=True)
    os.chdir(build_directory)
    if not kwargs['make']:
        args_str = ' '.join([f'{key}={value}' for key, value in cmake_args.items()])
        cmd = 'cmake ' + args_str + ' ..'
        LOG_INFO(cmd)
        execute_shell(cmd)
    # make -j
    cmd = 'make -j{}'.format(jobs)
    if verbose:
        cmd += " VERBOSE=1"
    cmd += " && make install"
    LOG_INFO(cmd)
    execute_shell(cmd)
    elapsed_time = datetime.datetime.now() - date_s
    LOG_INFO("Build finished, elapse ", str(elapsed_time))
    os.chdir(workspace)

    for (pkg, pkg_dir, pkg_lib) in zip(PKG_ALIAS, PKG_DIRS, PKG_LIBS):
        if kwargs[pkg]:
            lib_name = "lib"+pkg_lib+".so"
            copy_file(release_directory+"/"+platform+"/lib/"+lib_name, workspace+"/depend/"+pkg_dir+"/release/"+platform+"/lib")

def mdc_build(workspace, platform, build_directory, release_directory, **kwargs):
    # download release package
    sp.run('bash tools/downloadPkg.sh mdc', shell=1)

    # 设置环境变量
    if kwargs['gcc']:
        set_env('PATH', '/usr/local/mdc_sdk/dp_gea/mdc_cross_compiler/bin')
        set_env('CC', 'aarch64-target-linux-gnu-gcc')
        set_env('CXX', 'aarch64-target-linux-gnu-g++')
    else:
        set_env('PATH', '/usr/local/mdc_sdk_llvm/dp_gea/mdc_cross_compiler/bin')
        set_env('CC', 'clang')
        set_env('CXX', 'clang++')
        set_env('LD_LIBRARY_PATH', '/usr/local/mdc_sdk_llvm/dp_gea/mdc_cross_compiler/sysroot/usr/lib64')

    # 设置cmake编译选项
    args = dict()
    args['-DCMAKE_INSTALL_PREFIX'] = release_directory + "/mal_mdc"
    args['-DCMAKE_BUILD_TYPE'] = "Release" if kwargs['release'] else "Debug"
    args['-DPLATFORM'] = 'mdc'
    args['-DMAPPING_SINGLE_MODULE_COMPILE'] = 'ON'
    args['-DMAPPING_LIB_PREFIX'] = kwargs['prefix']
    args['-DCMAKE_EXPORT_COMPILE_COMMANDS'] = '1'
    args['-DMIDDLEWARE'] = "ADF"
    args['-DIND'] = "ON" if kwargs['ind'] else "OFF"
    args['-DHDMAP'] = kwargs['hdmap']
    for (pkg, pkg_cmake_enable) in zip(PKG_ALIAS, PKG_CMAKE_ENABLES):
        args[pkg_cmake_enable] = 'ON' if kwargs[pkg] else "OFF"
    # args['-DENABLE_COMPILE_BASE'] = 'ON' if kwargs['base'] else "OFF"
    # args['-DENABLE_UT'] = 'FLASE' if not kwargs['ut'] else 'TRUE'
    cmake_build(workspace, platform, build_directory, args, kwargs['jobs'], kwargs['verbose'])

def x86_build(workspace, platform, build_directory, release_directory, **kwargs):
    """x86 编译流程"""
    sp.run('bash tools/downloadPkg.sh x86', shell=1)
    set_env('PATH', '/usr/bin')
    set_env('CC', '/usr/bin/x86_64-linux-gnu-gcc')
    set_env('CXX', '/usr/bin/x86_64-linux-gnu-g++')

    set_env('LD_LIBRARY_PATH', osp.join(workspace, 'depend/third_party/x86/protobuf/lib'))
    args = dict()
    args['-DCMAKE_INSTALL_PREFIX'] = release_directory+"/mal_x86"
    args['-DCMAKE_BUILD_TYPE'] = "Release" if kwargs['release'] else "Debug"
    args['-DPLATFORM'] = 'x86_2004'
    # args['-DENABLE_UT'] = 'FLASE' if not kwargs['ut'] else 'TRUE'
    args['-DMAPPING_SINGLE_MODULE_COMPILE'] = 'ON'
    args['-DMAPPING_LIB_PREFIX'] = kwargs['prefix']
    args['-DCMAKE_EXPORT_COMPILE_COMMANDS'] = '1'
    args['-DIND'] = "ON" if kwargs['ind'] else "OFF"
    args['-DMIDDLEWARE'] = "CYBER" if kwargs['cyber'] else "LITE"
    args['-DHDMAP'] = kwargs['hdmap']
    for (pkg, pkg_cmake_enable) in zip(PKG_ALIAS, PKG_CMAKE_ENABLES):
        args[pkg_cmake_enable] = 'ON' if kwargs[pkg] else "OFF"
    # args['-DENABLE_COMPILE_BASE'] = 'ON' if kwargs['base'] else "OFF"

    cmake_build(workspace, platform, build_directory, args, kwargs['jobs'], kwargs['verbose'])

def orin_build(workspace, platform, build_directory, release_directory, **kwargs):
    """orin 编译流程"""
    set_env('PATH', '/usr/local/orin_sdk/aarch64/bin')
    set_env('CC', 'aarch64-linux-gcc')
    set_env('CXX', 'aarch64-linux-g++')
    LOG_INFO("Start update submodule")
    # sync submodule version
    execute_shell("git submodule update --init")
    # download release package
    execute_shell('bash tools/downloadPkg.sh orin')
    # cmake param set
    args = dict()
    args['-DCMAKE_INSTALL_PREFIX'] = release_directory+"/mal_orin"
    args['-DCMAKE_BUILD_TYPE'] = "Release" if kwargs['release'] else "Debug"
    args['-DPLATFORM'] = 'orin'
    # args['-DENABLE_UT'] = 'FLASE' if not kwargs['ut'] else 'TRUE'
    args['-DMAPPING_SINGLE_MODULE_COMPILE'] = 'ON'
    args['-DMAPPING_LIB_PREFIX'] = kwargs['prefix']
    args['-DCMAKE_EXPORT_COMPILE_COMMANDS'] = '1'
    args['-DMIDDLEWARE'] = "LITE"
    args['-DIND'] = "ON" if kwargs['ind'] else "OFF"
    if kwargs['plugin']:
        os.environ['WITH_MAL_PLUGIN_FLAG'] = 'true'
    else:
        os.environ['WITH_MAL_PLUGIN_FLAG'] = 'false'
    with open('plugin_env.txt', 'w') as file:
            file.write(os.environ.get('WITH_MAL_PLUGIN_FLAG', ''))
    execute_shell('echo ${WITH_MAL_PLUGIN_FLAG}')
    args['-DHDMAP'] = kwargs['hdmap']
    for (pkg, pkg_cmake_enable) in zip(PKG_ALIAS, PKG_CMAKE_ENABLES):
        args[pkg_cmake_enable] = 'ON' if kwargs[pkg] else "OFF"
    # args['-DENABLE_COMPILE_BASE'] = 'ON' if kwargs['base'] else "OFF"
    cmake_build(workspace, platform, build_directory, args, kwargs['jobs'], kwargs['verbose'])

def copy_header_files_by_path(src_path, dst_path):
    header_files = []
    for root, dirs, files in os.walk(src_path):
        dirs[:] = [d for d in dirs if d not in ['depend', 'release', 'Debug', 'Release', 'build', 'test']]  # 排除文件夹
        for file in files:
            if file.endswith('.h') or file.endswith('.hpp'):
                header_files.append(os.path.join(root, file))

    for file in header_files:
        file_path_in_dst_folder = os.path.join(dst_path, os.path.relpath(file, start=src_path))
        if not os.path.exists(os.path.dirname(file_path_in_dst_folder)):
            os.makedirs(os.path.dirname(file_path_in_dst_folder))
        shutil.copy(file, file_path_in_dst_folder)

def find_header_files_in_directory(dir_path):
    header_files = []
    for entry in os.scandir(dir_path):
        if entry.is_dir(follow_symlinks=False):
            header_files += find_header_files_in_directory(entry.path)
        elif entry.is_file(follow_symlinks=False):
            if entry.name.endswith('.h') or entry.name.endswith('.hpp'):
                header_files.append(entry.path)
    return header_files

def delete_empty_dir(dir):
    dir_list = []
    for root,dirs,files in os.walk(dir):
        dir_list.append(root)
    # 先生成文件夹的列表，重点是下边
    for root in dir_list[::-1]:
        if not os.listdir(root):
            os.rmdir(root)

def start_copy_head(platform):
    # 开始copy 头文件
    if platform=="x86":
        head_src_path = os.getcwd()
        LOG_INFO(head_src_path)
        head_target_path = release_directory+"/x86/include/mapping/"
        LOG_INFO(head_target_path)
        copy_header_files_by_path(head_src_path, head_target_path)
    elif platform=="mdc":
        head_src_path = os.getcwd()
        head_target_path = release_directory+"/mdc/mapping/include/"
        copy_header_files_by_path(head_src_path, head_target_path)
    elif platform=="orin":
        head_src_path = os.getcwd()
        head_target_path = release_directory+"/orin/include/mapping/"
        LOG_INFO(head_target_path)
        copy_header_files_by_path(head_src_path, head_target_path)

def make_package(platform):
    start_copy_head(platform)

def build_rviz_bridge(workspace, jobs):
    LOG_INFO("Build rviz_bridge")
    build_dir = osp.join(workspace, 'tools', 'rviz_bridge', 'build')
    os.makedirs(build_dir, exist_ok=True)
    os.chdir(build_dir)
    cmd = 'cmake .. && make -j{} && make install'.format(jobs)
    execute_shell(cmd)
    os.chdir(workspace)

def build_mapping_tool(workspace, release_directory, jobs):
    LOG_INFO("Build Mapping_tools")
    build_dir = osp.join(workspace, 'depend', 'Mapping_tools', 'build')
    os.makedirs(build_dir, exist_ok=True)
    os.chdir(build_dir)
    proto_dir = osp.join(workspace, 'depend', 'proto')
    proto_lib_dir = osp.join(release_directory, 'mal_x86/lib')
    third_party_dir = osp.join(workspace, 'depend', 'third_party')
    ap_release_dir = osp.join(workspace, 'depend', 'ap-release')
    install_dir = osp.join(release_directory, 'Mapping_tools')
    cmake_args = '-DPROTO_DIR={} '.format(proto_dir)
    cmake_args += '-DPROTO_LIB_DIR={} '.format(proto_lib_dir)
    cmake_args += '-DTHIRD_PARTY_DIR={} '.format(third_party_dir)
    cmake_args += '-DAP_RELEASE_DIR={} '.format(ap_release_dir)
    cmake_args += '-DCMAKE_INSTALL_PREFIX={}'.format(install_dir)
    cmd = 'cmake {} .. && make -j{} && make install'.format(cmake_args, jobs)
    LOG_INFO("Build cmd: {}".format(cmd))
    execute_shell(cmd)
    os.chdir(workspace)

def clean(workspace):
    build_list=["Debug", "Release", "release", "build", "output"]
    for dir in build_list:
        path=os.path.join(workspace,dir)
        if os.path.exists(path):
            execute_shell("rm -r {}".format(path))
    # 移除proto生成的.h和.cc文件
    execute_shell("find . -name *.pb.h | grep -v 'depend/third_party' |xargs rm -rf")
    execute_shell("find . -name *.pb.cc | grep -v 'depend/third_party' |xargs rm -rf")
    execute_shell("find . -name *.om | grep -v 'depend/third_party' |xargs rm -rf")
    LOG_INFO("delete cmake build production success.")

def all_build(workspace, platform, build_directory, release_directory, **kwargs):
    sp.run('rm -rf {}'.format(build_directory), shell=1)
    sp.run('git submodule update --init', shell=True)
    x86_build(workspace, platform, build_directory, release_directory, **kwargs)
    # start_copy_head('x86')

    sp.run('rm -rf {}'.format(build_directory), shell=1)
    mdc_build(workspace, platform, build_directory, release_directory, **kwargs)
    # start_copy_head('mdc')

    sp.run('rm -rf {}'.format(build_directory), shell=1)
    orin_build(workspace, platform, build_directory, release_directory, **kwargs)
    # start_copy_head('orin')

if __name__ == '__main__':
    # 解析编译选项为dict
    kwargs = vars(parse_args())
    LOG_INFO('compile option:', kwargs)
    workspace = kwargs.pop('workspace')
    os.chdir(workspace)
    build_directory = osp.join(workspace, 'build')
    release_directory = osp.join(workspace, 'release')
    platform = kwargs.pop('platform')
    sp.run('git submodule update --init', shell=True)

    # 清理cmake编译缓存
    if kwargs['clean']:
        clean(workspace)
    # 构建rviz_bridge
    if kwargs['rviz']:
        build_rviz_bridge(workspace, kwargs['jobs'])

    # 开始构建工程
    if platform == 'mdc':
        mdc_build(workspace, platform, build_directory, release_directory, **kwargs)
        # 开始copy 头文件
        # make_package(platform)
    elif platform == 'x86':
        x86_build(workspace, platform, build_directory, release_directory, **kwargs)
        # make_package(platform)
        if kwargs['tool']:
            if kwargs['cyber']:
                build_mapping_tool(workspace, release_directory, kwargs['jobs'])
            else:
                LOG_ERROR('--tool should be used with --cyber')
    elif platform == 'orin':
        orin_build(workspace, platform, build_directory, release_directory, **kwargs)
        # make_package(platform)
    elif platform == 'all':
        all_build(workspace, platform, build_directory, release_directory, **kwargs)

    del_remain_external_lib(workspace, platform, release_directory, **kwargs)

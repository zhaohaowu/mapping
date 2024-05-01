#!/bin/bash

# 显示脚本的使用方法
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  -c, --count   Specify the number of commits (default: 5)"
    echo "  -t, --tag     Filter commits by tag"
    echo "  -h, --help    Display this help message"
    exit 0
}

# 默认提交数量和标签
# commit_count=2
tag=""
tag_name=""

# 解析参数
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        -c|--count)
        commit_count="$2"
        shift
        shift
        ;;
        count)
        commit_count="$2"
        shift
        shift
        ;;
        -t|--tag)
        tag="$2"
        shift
        shift
        ;;
        tag)
        tag="$2"
        shift
        shift
        ;;
        -h|--help)
        show_usage
        ;;
        help)
        show_usage
        ;;
        *)
        echo "Unknown option: $1"
        exit 1
        ;;
    esac
done

# 如果没有提供标签，也无法从 version.json 中获取，则直接返回
if [ -z "$commit_count" ]; then
    echo "No commit count provided. please using -c N or --count N"
    exit 0
fi

# 检查是否提供了标签
if [ -z "$tag" ]; then
    # 从 version.json 中获取标签名的逻辑
    version_info=$(cat version.json)
    release_version=$(echo $version_info | jq -r '.ORIN.EP41.releaseVersion')
    echo "release_version: ${release_version}"

    date_part=$(echo $release_version | awk -F '_' '{print $4"-"$5"-"$6}')
    echo "date_part: ${date_part}"


    commit_part=$(echo $release_version | awk -F '_' '{print $7}')
    echo "commit_part: ${commit_part}"

    tag_name="v$date_part-$commit_part"
else
    tag_name="$tag"
fi

# 如果没有提供标签，也无法从 version.json 中获取，则直接返回
if [ -z "$tag_name" ]; then
    echo "No tag provided and unable to determine tag from version.json. Exiting without creating tag."
    exit 0
fi

# 获取前N个提交的提交消息
commit_messages=$(git log -n $commit_count --pretty=format:"%s" | sed 's/URL:.*$//' | tail -n +2 | awk '{print NR". "$0}')
echo "$commit_messages"

# 检查是否存在重名的标签
if git rev-parse "$tag_name^{tag}" >/dev/null 2>&1; then
    echo "Error: Tag $tag_name already exists."
    exit 1
fi

# 创建新的 Git 标签
if [ -n "$commit_messages" ]; then
    if [ -n "$tag_name" ]; then
        git tag -a $tag_name -m "$commit_messages"
        echo "Created tag $tag_name with the following commit messages:"

        # 将标签推送到远程仓库
        git push origin $tag_name
        echo "Pushed tag $tag_name to remote repository."
    else
        echo "No tag provided. Exiting without creating tag."
    fi
else
    echo "No commit messages found to create tag."
fi


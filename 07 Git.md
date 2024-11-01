# 1 Git安装
    sudo apt-get install git

# 2 Git配置
    git config --global user.name  “aaa”
    git config --global user.email "aa@qq.com"
    ssh-keygen -t rsa -C "aa@qq.com"

    cd ~/.ssh
    gedit id_rsa.pub
将里面的公钥内容进行复制，贴到浏览器中！！
    ssh -T git@git.oschina.net
测试是否可以连接通，输出：Permission denied (publickey)

# 3 检查 并添加远程仓库
    git remote -v
    git remote set-url origin git@github.com:flyingbear24/GitTest1030.git

# 4 本地提交
创建并切换到分支：
    git checkout -b branch_name
删除分支
    git branch -d branch_name

    sudo git add .
    sudo git commit -m "提交的备注"
    sudo git push origin branch_name

# 5 git拉取 克隆
    sudo mkdir test01
    cd test01
    git clone git@github.com:flyingbear24/GitTest1030.git

## 其他操作命令
    git status
    git log
    git branch
    git checkout branch_name
    rm -rf .git
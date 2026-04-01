查看会话 tmux a -t rover 
ctrl+b后 1 2 3 切换会话窗口 ，d是退出
关闭会话 tmux kill-session -t rover 
查看会话是否运行  tmux ls


开启系统后台服务 sudo systemctl start robot_core.service  
关闭系统后台服务 sudo systemctl stop robot_core.service
开启开机自启 sudo systemctl enable robot_core.service
取消开机自启 sudo systemctl disable robot_core.service
看系统自启动服务有没有正常跑 sudo systemctl status robot_core.service


创建虚拟串口对：终端中输入 socat -d -d pty,raw,echo=0 pty,raw,echo=0

使用物理接口时要修改串口桥接节点监听的端口为：/dev/ttyGS0
export RTT_EXEC_PATH=/home/robot/workspace/feizou/code/fmt-gitee/gcc-arm-none-eabi-7-2018-q2-update/bin
# scons -c
export errorNum=`scons |grep "building[[:space:]]terminated[[:space:]]because[[:space:]]of[[:space:]]errors"|wc -l`
if [ $errorNum -le 0 ];then
echo "***************************************compille is ok*************************************************************"
cp build/fmt_chuanyun_f2s.bin package_bin/
cd package_bin/
./joint2flash_cf.sh
# cp -f fmt_chuangyun_f2s_pkg.bin ../../../../
cd ..
python3 serialuploadrc.py
else
echo "***************************************something is wrong*************************************************************"
scons
fi


# quadqopter

## 运行说明

只能使用python2运行

````
python MainController.py
````



依赖项安装：

````
sudo apt-get install python-tk
pip install matplotlib
pip install pillow
pip install python-opencv
````

##### lua脚本

用`quadcopter.lua`内容覆盖quadcopter的脚本

用`jacoHand.lua`内容覆盖jacoHand的脚本

##### move_to函数

+ 若需要精确移动到pos，使用move_to(pos)

+ 若不需要精确性，使用move_to(pos,False)来节省时间
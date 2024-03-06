NES Arduino/Micro Pro adapter with an emphasis on Tetris Effect performance and navigation controls


### Tips for improving TEC latency<br> 
  -Turn off v-sync and in fullscreen mode<br> 
  -Turn off adaptive sync(gsync/freesync)<br> 
  -Put settings to lowest(partical size to maximum), and increase until latency becomes problematic<br> 
  -If using a multi-usb port, ensure other ports are empty<br> 
  -Restart computer if misdrops get worse over time(or take a break)<br> 
  -Turn off any power savings settings.  Prefer any performance mode over quality/power savings. e.g. nvidia settings, windows power settings, manufacturer oc/performance settings<br> 
  -If using nvidia on windows, enable in-game overlay in geforce experience app, open overlay, click performance, click gear, choose screen position, trying getting the render latnecy statstic low as possible.<br> 

### Streaming:<br> 
  -If OBS causes latency and spare device is available, try using hdmi to usb(dual monitor or laptop required).<br> 
  -Native NVidia streaming software cause significanly less latency.<br> 

### For all one of me playing TEC on linux:<br> 
  -nvidia-settings->PowerMizer->set preferred mode to prefer maximum performance and other settings to performance<br> 
  -Disable x11 compositor.  e.g. with steam's launch options:<br> 
    "gsettings set org.mate.Marco.general compositing-manager false;  %command%; gsettings set org.mate.Marco.general compositing-manager true"<br> 
  -Use a light weight desktop. e.g. mate, xfce, et al<br> 
  -Install a low latency kernel<br> 
  -Install gamemode<br> 
    e.g. steam launch options: gamemoderun %command%<br> 


for build instructions check out https://github.com/alex-ong/LaglessNESUSB

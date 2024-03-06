NES Arduino/Micro Pro adapter with an emphasis on Tetris Effect performance and navigation controls


### Tips for improving TEC latency
  *Turn off v-sync and in fullscreen mode
  *Turn off adaptive sync(gsync/freesync)
  *Put settings to lowest(partical size to maximum), and increase until latency becomes problematic
  *If using a multi-usb port, ensure other ports are empty
  *Restart computer if misdrops get worse over time(or take a break)
  *Turn off any power savings settings.  Prefer any performance mode over quality/power savings. e.g. nvidia settings, windows power settings, manufacturer oc/performance settings
  *If using nvidia on windows, enable in-game overlay in geforce experience app, open overlay, click performance, click gear, choose screen position, trying getting the render latnecy statstic low as possible.

### Streaming:
  *If OBS causes latency and spare device is available, try using hdmi to usb(dual monitor or laptop required).
  *Native NVidia streaming software cause significanly less latency.

### For all one of me playing TEC on linux:
  *nvidia-settings->PowerMizer->set preferred mode to prefer maximum performance and other settings to performance
  *Disable x11 compositor.  e.g. with steam's launch options:
    "gsettings set org.mate.Marco.general compositing-manager false;  %command%; gsettings set org.mate.Marco.general compositing-manager true"
  *Use a light weight desktop. e.g. mate, xfce, et al
  *Install a low latency kernel
  *Install gamemode
    e.g. steam launch options: gamemoderun %command%


for build instructions check out https://github.com/alex-ong/LaglessNESUSB

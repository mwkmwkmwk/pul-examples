set -e
export LD_PRELOAD=/home/mwk/prjleuctra/leuctra/ise/fixuseafterfree.so
xst -ifn ctr.xst
ngdbuild ctr -uc ctr.ucf
map ctr
par -w ctr.ncd ctr_par.ncd
bitgen -w ctr_par.ncd ctr_par.bit -g StartupClk:JTAGClk

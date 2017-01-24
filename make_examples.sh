bash clean.sh
clear

cd examples/FIFO
for dir in *
do 
	cd ${dir} 
        make
	cd scots-files
	make
        cd ..
	cd ..
done

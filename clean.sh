find ./ -type f -name "*.bdd" -delete
find ./ -type f -name "*.nbdd" -delete


cd examples/FIFO
for dir in *
do 
	cd ${dir} 
        make clean
	cd scots-files
	make clean
        cd ..
	cd ..
done

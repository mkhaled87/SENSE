find ./ -type f -name "*.bdd" -delete
find ./ -type f -name "*.nbdd" -delete
find ./ -type f -name "*.csv" -delete
find ./ -type f -name "*.fsm" -delete
find ./ -type f -name "*.scs" -delete
find ./ -type f -name "*.o" -delete


cd examples/prolonged_ncs
for dir in *
do 
	cd ${dir} 
        make clean
		cd scots-files
			make clean
		cd ..
	cd ..
done


cd ../prolonged_ncs_nondet
for dir in *
do 
	cd ${dir} 
        make clean
	cd scots-files
	make clean
        cd ..
	cd ..
done

cd ../others
for dir in *
do 
	cd ${dir} 
        make clean
	cd scots-files
	make clean
        cd ..
	cd ..
done
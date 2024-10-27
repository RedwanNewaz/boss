
WAF_EXE='/Users/redwan/CppDev/limbo/build/exp/boss2/boss2'
CMAKE_EXE='./boss'
# Parse command line options
# Check if an argument was passed to the script
if [ "$#" -eq 0 ]; then
    echo "[+] running cmake executable"
    $CMAKE_EXE --config ../test/env2.yaml --verbose 1
else
    echo "[+] running waf executable"
    $WAF_EXE --config ../test/env2.yaml --verbose 1
fi

./plot_boss2.py --env_file="../test/env2.yaml" 




commands to run:
cmake -S VSM/tests -B build/vsm-state-tests
cmake --build build/vsm-state-tests 
ctest --test-dir build/vsm-state-tests --output-on-failure 
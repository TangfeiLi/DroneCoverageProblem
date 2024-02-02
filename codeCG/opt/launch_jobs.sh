#!/bin/bash

cmdline_args=($*)
base_dir="D:/LTF/Postgraduate/Project/DiplomaProject/BigPaper"

for file in $(printf "%s%s%s%s" "${base_dir}" "/data/input/" "${cmdline_args[0]}" "*.txt")
do
  base_name=$(basename $file)
  grep "${base_name}" solved_instances.txt
  return_code=$?

  if [[ "${return_code}" == "1" ]]
  then
    echo "Launching job for ${file}"
    ${base_dir}/bin/Drone_cover.exe ${base_dir}/data/program_params.json ${base_dir}/data/result/results.txt ${base_dir}/data/input/${base_name} 2> ${base_dir}/data/.err/${base_name}.err > ${base_dir}/data/.out/${base_name}.out
    # oarsub -n "${base_name}" -O "${base_name}.out" -E "${base_name}.err" --p "network_address!='drbl10-201-201-21'" -l /nodes=1/core=2,walltime=10 "LD_LIBRARY_PATH=~/local/lib64 ${base_dir}/build/maritime_vrp ${base_dir}/data/program_params.json ${base_dir}/data/new/${base_name}"
    echo "${base_name}" >> solved_instances.txt
  else
    echo "Skipping job for ${file}"
  fi
done

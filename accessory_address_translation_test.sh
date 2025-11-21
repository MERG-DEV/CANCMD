#!/bin/bash
mock_headers()
{
  for header_to_mock in $*
  do
    if [[ ! -e ${header_to_mock} ]]
    then
      touch ${header_to_mock} $(basename ${header_to_mock} .h).mocked
    fi
  done
}

link_headers()
{
  for header_link in $*
  do
    IFS=","
    set -- ${header_link}
    source=${1}
    target=${2}
    if [[ ${2} == "." ]]
    then
      target=$(basename ${source})
    fi
    if [[ ! -e ${target} ]]
    then
      ln -s ${source} ${target}
    fi
  done
}

cleanup_headers()
{
  for header_to_clean in $*
  do
    if [[ -f $(basename ${header_to_clean} .h).mocked ]]
    then
      rm ${header_to_clean} $(basename ${header_to_clean} .h).mocked
    elif [[ -L ${header_to_clean} ]]
    then
      rm ${header_to_clean}
    fi
  done
}

mock_headers devincs.h GenericTypeDefs.h
link_headers FliM.h,FLiM.h ../cbusdefs/cbusdefs.h,.
gcc -ggdb -O0 -I. accessory_address_translation_test.c && ./a.out | tee accessory_address_translation_test.log
#gcc -ggdb -O0 -I. accessory_address_translation_test.c && ddd ./a.out
cleanup_headers devincs.h GenericTypeDefs.h FLiM.h cbusdefs.h

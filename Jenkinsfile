pipeline {
  agent any
  stages {
    stage('build') {
      parallel {
        stage('hello world sanity check') {
          steps {
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) samples/hello_world mimxrt1060_evk'
            sh '''docker exec confident_sinoussi run_setup.sh $(basename $(pwd))'''
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) samples/hello_world'
          }
        }
      }
    }
    stage('test') {
       parallel {
        stage('hello world') {
          steps {
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) samples/hello_world mimxrt1060_evk'
            sh '''docker exec confident_sinoussi run_setup.sh $(basename $(pwd))'''
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) samples/hello_world'
          }
        }

    }
  }
}

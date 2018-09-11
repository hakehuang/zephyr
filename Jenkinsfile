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
    stage('test suite') {
      parallel {
      stage('mem_protect_protection') {
          steps {
            echo 'mem_protect_protection'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/kernel/mem_protect/protection mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/kernel/mem_protect/protection mem_protect_protection'
          }
        }
      stage('xip') {
          steps {
            echo 'xip'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/kernel/xip mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/kernel/xip xip'
          }
        }
      stage('obj_validation') {
          steps {
            echo 'obj_validation'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/kernel/mem_protect/obj_validation mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/kernel/mem_protect/obj_validation obj_validation'
          }
        }
      stage('mpu_test') {
          steps {
            echo 'mpu_test'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) samples/mpu/mpu_test mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) samples/mpu/mpu_test mpu_test'
          }
        }
        stage('footprint') {
          steps {
            echo 'footprint'
            sh '''docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/benchmarks/footprint mimxrt1060_evk
'''
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/benchmarks/footprint footprint'
          }
        }
        stage('mbedtls') {
          steps {
            echo 'mbedtls'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/crypto/mbedtls mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/crypto/mbedtls mbedtls'
          }
        }   
        stage('entropy') {
          steps {
            echo 'entropy'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) samples/drivers/entropy mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) samples/drivers/entropy  entropy'
          }
        }
        stage('tests_drivers_ipm') {
          steps {
            echo 'tests_drivers_ipm'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/drivers/ipm mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/drivers/ipm tests_drivers_ipm '
          }
        }
        stage('uart_basic_api') {
          steps {
            echo 'uart_basic_api'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/drivers/uart/uart_basic_api mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/drivers/uart/uart_basic_api uart_basic_api'
            echo 'uart_basic_pool'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/drivers/uart/uart_basic_api mimxrt1060_evk -DCONF_FILE=prj_poll.conf'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/drivers/uart/uart_basic_api uart_basic_poll'
            echo 'uart_basic_shell '
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/drivers/uart/uart_basic_api mimxrt1060_evk -DCONF_FILE=prj_shell.conf'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/drivers/uart/uart_basic_api uart_basic_shell'
          }
        }
        stage('rand32') {
          steps {
            echo 'rand32'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/crypto/rand32 mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/crypto/rand32  rand32'
          }
        }
        stage('kernel_thread') {
          steps {
            echo 'kernel_thread'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/kernel/threads/no-multithreading mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/kernel/threads/no-multithreading kernel_thread'
          }
        }
        stage('latency_measure') {
          steps {
            echo 'latency_measure'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/benchmarks/latency_measure mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/benchmarks/latency_measure latency_measure'
          }
        }
        stage('sched_preempt') {
          steps {
            echo 'sched_preempt'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/kernel/sched/preempt mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/kernel/sched/preempt sched_preempt'
          }
        }
        stage('arm_irq_vector_table') {
          steps {
            echo 'arm_irq_vector_table'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/kernel/arm_irq_vector_table mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/kernel/arm_irq_vector_table  arm_irq_vector_table'
          }
        }
        stage('json') {
          steps {
            echo 'json_test'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/lib/json mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/lib/json  json'
          }
        }
        stage('boot_time') {
          steps {
            echo 'boot_time'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/benchmarks/boot_time mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/benchmarks/boot_time boot_time'
          }
        }
        stage('kernel_fatal') {
          steps {
            echo 'kernel_fatal'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/kernel/fatal mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/kernel/fatal kernel_fatal'
          }
        }
        stage('entropy_api') {
          steps {
            echo 'entropy_api'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/drivers/entropy/api mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/drivers/entropy/api entropy_api '
          }
        }
        stage('getline') {
          steps {
            echo 'getline'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) samples/subsys/console/getline mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) samples/subsys/console/getline getline'
          }
        }
        stage('shell') {
          steps {
            echo 'Test shell'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/shell/ mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/shell/ shell'
          }
        }
        stage('getchar') {
          steps {
            echo 'getchar'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) samples/subsys/console/getchar mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) samples/subsys/console/getchar getchar'
          }
        }
        stage('mpu_stack_guard_test') {
          steps {
            echo 'mpu_stack_guard_test'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) samples/mpu/mpu_stack_guard_test mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) samples/mpu/mpu_stack_guard_test  mpu_stack_guard_test'
            echo 'mpu_stack_guard_test_enable_stack_guard'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) samples/mpu/mpu_stack_guard_test mimxrt1060_evk  -DCONF_FILE=prj_stack_guard.conf'
            sh 'docker exec confident_sinoussi run_zephyr_elf.sh $(basename $(pwd)) samples/mpu/mpu_stack_guard_test  mpu_stack_guard_test_enabled'

          }
        }
        stage('shell_module') {
          steps {
            echo 'shell_module'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) samples/subsys/shell/shell_module mimxrt1060_evk'
sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) samples/subsys/shell/shell_module shell_module'
          }
        }
        stage('gen_isr_table') {
          steps {
            echo 'gen_isr_table'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/kernel/gen_isr_table mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/kernel/gen_isr_table  gen_isr_table'
          }
        }
        stage('mem_domain_apis_test') {
          steps {
            echo 'mem_domain_apis_test'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) samples/mpu/mem_domain_apis_test mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) samples/mpu/mem_domain_apis_test  mem_domain_apis_test'
          }
        }
        stage('tickless') {
          steps {
            echo 'tickless'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/kernel/tickless/tickless mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/kernel/tickless/tickless tickless'
          }
        }
        stage('ipm_mailbox_ap') {
          steps {
            echo 'ipm_mailbox_ap'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) samples/subsys/ipc/ipm_mailbox/ap mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) samples/subsys/ipc/ipm_mailbox/ap ipm_mailbox_ap '
          }
        }
        stage('nfc_hello') {
          steps {
            echo 'nfc_hello'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) samples/nfc/nfc_hello mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) samples/nfc/nfc_hello nfc_hello'
          }
        }
        stage('mem_protect_userspace') {
          steps {
            echo 'mem_protect_userspace'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) tests/kernel/mem_protect/userspace mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) tests/kernel/mem_protect/userspace tickless'
          }
        }
        stage('sysview') {
          steps {
            echo 'sysview'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) samples/subsys/debug/sysview mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) samples/subsys/debug/sysview sysview'
          }
        }
        stage('sensor') {
          steps {
            echo 'sensor'
            sh 'docker exec confident_sinoussi build_zephyr_elf.sh  $(basename $(pwd)) samples/subsys/ipc/ipm_mailbox/sensor mimxrt1060_evk'
            sh 'docker exec confident_sinoussi run_zephyr_elf_rt1060.sh $(basename $(pwd)) samples/subsys/ipc/ipm_mailbox/sensor sensor'
          }
        }
      }        
    }
    stage('mail') {
      steps {
        mail(subject: 'zephyr_rt1050', body: 'zephyr_rt1050 CI PASS notification', from: 'zephyr_ci', to: 'hake.huang@nxp.com')
      }
    }
  }
}

// Jenkins declarative pipeline for RoboticsCore (the project's only CI system, see
// docs/adr/0006-continuous-integration.md). Runs on Linux or Windows agents; every step goes
// through run(), which dispatches to sh or bat. Each stage maps to commands documented in
// CONTRIBUTING.md, so failures can be reproduced locally.

// Run a shell command. On Windows the Visual Studio developer environment is loaded first.
def run(String cmd) {
  if (isUnix()) {
    sh cmd
  } else {
    bat "call scripts\\setup\\vs_env.cmd && ${cmd}"
  }
}

def py() { return isUnix() ? '.venv/bin/python' : '.venv\\Scripts\\python.exe' }

pipeline {
  agent { label params.AGENT_LABEL }

  options {
    disableConcurrentBuilds()
    timestamps()
    timeout(time: 90, unit: 'MINUTES')
    buildDiscarder(logRotator(numToKeepStr: '20', artifactNumToKeepStr: '5'))
  }

  parameters {
    string(name: 'AGENT_LABEL', defaultValue: 'linux', description: 'Agent label (linux or windows)')
    booleanParam(name: 'RUN_SANITIZERS', defaultValue: true, description: 'ASan + UBSan test run (GCC/Clang agents)')
    booleanParam(name: 'RUN_CLANG_TIDY', defaultValue: true, description: 'Build with clang-tidy enabled')
    booleanParam(name: 'RUN_BENCHMARKS', defaultValue: false, description: 'Run benchmarks and archive results')
    booleanParam(name: 'RENDER_DOCS', defaultValue: false, description: 'Re-render documentation animations')
  }

  environment {
    CMAKE_BUILD_PARALLEL_LEVEL = '4'
    CTEST_PARALLEL_LEVEL = '4'
    PYTHONPATH = 'python'
  }

  stages {
    stage('Toolchain') {
      steps {
        run 'cmake --version'
        run 'ninja --version'
        script {
          if (isUnix()) {
            sh 'c++ --version || true'
          }
        }
      }
    }

    stage('Python environment') {
      steps {
        script {
          if (isUnix()) {
            sh 'python3 -m venv .venv && .venv/bin/python -m pip install --quiet -e ".[dev]"'
          } else {
            bat 'python -m venv .venv && .venv\\Scripts\\python.exe -m pip install --quiet -e ".[dev]"'
          }
        }
      }
    }

    stage('Lint') {
      steps {
        run "${py()} -m ruff check python scripts"
        run "${py()} -m ruff format --check python scripts"
        run "${py()} scripts/docs/generate_status.py --check"
        run 'git ls-files "*.hpp" "*.cpp" > clang-format-files.txt'
        run 'clang-format --dry-run --Werror --files=clang-format-files.txt'
      }
    }

    stage('Build and test (Debug)') {
      steps {
        run 'cmake --preset ci-debug'
        run 'cmake --build --preset ci-debug'
        run 'ctest --preset ci-debug -L unit --output-junit junit-unit.xml'
        run 'ctest --preset ci-debug -L numerical --output-junit junit-numerical.xml'
        run 'ctest --preset ci-debug -L integration --output-junit junit-integration.xml'
        run 'ctest --preset ci-debug -L regression --output-junit junit-regression.xml'
      }
    }

    stage('Build and test (Release)') {
      steps {
        run 'cmake --preset ci-release'
        run 'cmake --build --preset ci-release'
        run 'ctest --preset ci-release --output-junit junit-release.xml'
      }
    }

    stage('Install and consume') {
      steps {
        run 'cmake --install build/ci-release --prefix build/install/ci-release'
        script {
          def prefix = "${env.WORKSPACE}/build/install/ci-release".replace('\\', '/')
          run "cmake -S examples/external_project -B build/external -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=${prefix}"
        }
        run 'cmake --build build/external'
        script {
          if (isUnix()) {
            sh './build/external/plan_and_track'
          } else {
            bat 'build\\external\\plan_and_track.exe'
          }
        }
      }
    }

    stage('Python tests') {
      steps {
        run "${py()} -m pytest -q"
      }
    }

    stage('Static analysis') {
      when { expression { params.RUN_CLANG_TIDY } }
      steps {
        run 'cmake --preset clang-debug -B build/tidy -DROBOTICSCORE_ENABLE_CLANG_TIDY=ON -DROBOTICSCORE_BUILD_TESTS=OFF -DROBOTICSCORE_BUILD_BENCHMARKS=OFF'
        run 'cmake --build build/tidy'
      }
    }

    stage('Sanitizers') {
      when { allOf { expression { params.RUN_SANITIZERS }; expression { isUnix() } } }
      steps {
        run 'cmake --preset sanitize'
        run 'cmake --build --preset sanitize'
        run 'ctest --preset sanitize --output-junit junit-sanitize.xml'
      }
    }

    stage('Benchmarks') {
      when { expression { params.RUN_BENCHMARKS } }
      steps {
        run "${py()} scripts/benchmark/run_benchmarks.py --build-dir build/ci-release"
        archiveArtifacts artifacts: 'results/*.json, docs/benchmarks/README.md', allowEmptyArchive: false
      }
    }

    stage('Render documentation') {
      when { expression { params.RENDER_DOCS } }
      steps {
        script {
          withEnv(["ROBOTICSCORE_DEMO=${isUnix() ? 'build/ci-release/apps/roboticscore_demo' : 'build\\ci-release\\apps\\roboticscore_demo.exe'}"]) {
            run "${py()} scripts/render/render_all.py"
          }
        }
        archiveArtifacts artifacts: 'docs/assets/*.gif, docs/assets/manifest.json'
      }
    }

    stage('Secret scan') {
      steps {
        script {
          if (isUnix()) {
            sh '''
              if command -v gitleaks >/dev/null 2>&1; then
                gitleaks detect --source . --redact --no-banner
              else
                echo "gitleaks not installed on this agent; skipping secret scan"
              fi
            '''
          } else {
            bat '''
              where gitleaks >nul 2>nul
              if errorlevel 1 (
                echo gitleaks not installed on this agent; skipping secret scan
                exit /b 0
              )
              gitleaks detect --source . --redact --no-banner
            '''
          }
        }
      }
    }
  }

  post {
    always {
      junit testResults: 'build/**/junit-*.xml', allowEmptyResults: true
    }
    cleanup {
      cleanWs(deleteDirs: true, notFailBuild: true)
    }
  }
}

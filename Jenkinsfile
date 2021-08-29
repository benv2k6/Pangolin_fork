pipeline {
    agent none

    stages {
        stage('Hello') {
            agent {
                docker {image 'node:16-alpine'}
                
            }
            steps {
                echo 'Hello World'
            }
        }
    }
}

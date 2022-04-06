1. install jekyll pre reqs: https://jekyllrb.com/docs/installation/other-linux/ (ruby 2.7)
2. Ensure Ruby 2.7.1 is installed. Make sure that your command line is being started as a login shell when executing the steps below

        sudo apt-get install rvm
        echo 'source /etc/profile.d/rvm.sh' >> ~/.bashrc
        echo 'export PATH=$HOME/gems/bin' >> ~/.bashrc
        source ~/.bashrc
        rvm install 2.7.1
        rvm use 2.7.1
  
3. Install gem dependencies

        cd /path/to/gym2real
        bundle install
        
4. Run the jeyll server

        bundle exec jekyll serve

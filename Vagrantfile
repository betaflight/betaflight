# -*- mode: ruby -*-
# vi: set ft=ruby :

# All Vagrant configuration is done below. The "2" in Vagrant.configure
# configures the configuration version (we support older styles for
# backwards compatibility). Please don't change it unless you know what
# you're doing.
Vagrant.configure(2) do |config|
  # The most common configuration options are documented and commented below.
  # For a complete reference, please see the online documentation at
  # https://docs.vagrantup.com.

  # Every Vagrant development environment requires a box. You can search for
  # boxes at https://app.vagrantup.com/boxes/search
  config.vm.box = "ubuntu/xenial64"

  config.vm.provider "virtualbox" do |v|
      v.memory = 4096
      v.customize [ "guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-interval", 10000]
      v.customize [ "guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-min-adjust", 100]
      v.customize [ "guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-on-restore", 1]
      v.customize [ "guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-threshold", 10000]
  end

  # Enable provisioning with a shell script. Additional provisioners such as
  # Puppet, Chef, Ansible, Salt, and Docker are also available. Please see the
  # documentation for more information about their specific syntax and use.
  config.vm.provision "shell", inline: <<-SHELL
    apt-get remove -y binutils-arm-none-eabi gcc-arm-none-eabi
    add-apt-repository ppa:team-gcc-arm-embedded/ppa
    apt-get update
    apt-get install -y git gcc-arm-embedded=6-2017q2-1~xenial1
    apt-get install -y make python gcc clang
    apt-get install -y libblocksruntime-dev
  SHELL
end

# Usage
# On windows start a command shell in the project root directory, where the "Vagrantfile" exists.
# "vagrant up"     to start the VM. First time it takes a while.....
# "vagrant ssh"    to log into your VM.
# "cd /vagrant"    Here are the windows project directory mounted with all your files.
# "make all"       Start working, building all targets for example.
# "exit"           when done
# vagrant halt     to stop your VM
# vagrant --help   for more....

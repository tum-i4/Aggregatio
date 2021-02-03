# Installing Oracle JDK 11 on Ubuntu 18.04

To install Oracle JDK 11 on Ubuntu 18.04, please follow the next steps.

First, make sure that your system is up to date, using the following command:

```
sudo apt update && sudo apt upgrade
```

Next, download the [Oracle JDK 11](https://www.oracle.com/java/technologies/javase-jdk11-downloads.html#license-lightbox) compressed archive. In our testbed we are using JDK 11.0.7 release.

Create a new folder in your cache directory and copy the downloaded archive:

```
sudo mkdir -p /var/cache/oracle-jdk11-installer-local/
sudo cp jdk-11.0.7_linux-x64_bin.tar.gz /var/cache/oracle-jdk11-installer-local/
```

Now, we need to add the Java's PPA to Ubuntu apt:
```
sudo add-apt-repository ppa:linuxuprising/java
sudo apt-get update
```

Next, install Oracle JDK 11:
```
sudo apt install oracle-java11-installer-local
```

Check whether the Java installation was successful:
```
java --version
```
You will see something like this:
```
java 11.0.7 2020-04-14 LTS
Java(TM) SE Runtime Environment 18.9 (build 11.0.7+8-LTS)
Java HotSpot(TM) 64-Bit Server VM 18.9 (build 11.0.7+8-LTS, mixed mode)
```

If you require a more comprehensive guide, you might want to read [A quick installation guide for Ubuntu Linux users installing Java 11](https://www.javaworld.com/article/3514725/installing-oracle-java-se-11-on-ubuntu-18-04.html) and [How to Install Oracle Java 11 in Ubuntu 18.04/18.10](http://ubuntuhandbook.org/index.php/2018/11/how-to-install-oracle-java-11-in-ubuntu-18-04-18-10/) guides.
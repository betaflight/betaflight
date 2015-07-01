#Travis

Cleanflight provides Travis build and config files in the repository root.

## Pushing builds to a remote server

```.travis.sh``` script can upload build artifacts to a remote server. This feature is controlled by the
```PUBLISH_URL``` environment variable. If set, the build script will use the cURL binary and simulate
a file upload post to the configured server.

Pleas check the ```notifications``` section in the ```.travis.yml``` file and adjust the irc notifications if you plan on using Travis on your Cleanflight fork

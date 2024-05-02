# Robot URDF generation

The URDF model for LRR is generated using a python tool called [OnShape to Robot](https://onshape-to-robot.readthedocs.io/en/latest/).

If you wish to generate your own files you'll need to read the docs and setup your own OnShape API keys.

I use the Linux package `sed` for replacing the `package://` based model addresses with `localhost` based addresses, which is required to play nice with the Foxglove web viewer. If you aren't using Linux, change `config.json` to omit `postImportCommands` and use some other tool to do the replacement.

See `SOFTWARE/docker/docker-compose.yml` for information on the nginx webserver that serves these files.

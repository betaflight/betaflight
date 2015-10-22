#
# Variables
#

NAME = Switchery

#
# Paths
#

COMPONENT_BUILD = node_modules/.bin/component-build
COMPONENT_INSTALL = node_modules/.bin/component-install
UGLIFYJS = node_modules/uglify-js/bin/uglifyjs
UGLIFYCSS = node_modules/uglifycss/uglifycss
JS_DEST = dist/switchery.js
JS_MIN_DEST = dist/switchery.min.js
CSS_DEST = dist/switchery.css
CSS_MIN_DEST = dist/switchery.min.css

#
# All
#

all: install

#
# Install
#

install: node_modules components build

#
# Make a new development build
#

build: components switchery.js switchery.css
	@$(COMPONENT_BUILD) --dev

#
# Install components (+ dev)
#

components: component.json
	@$(COMPONENT_INSTALL) --dev

#
# Make a standalone version that doesn't depend on component etc.
#

standalone: build components
	@$(COMPONENT_BUILD) -s $(NAME) -o .
	@mv build.js $(JS_DEST)
	@mv build.css $(CSS_DEST)
	@$(UGLIFYJS) $(JS_DEST) --output $(JS_MIN_DEST)
	@$(UGLIFYCSS) $(CSS_DEST) > $(CSS_MIN_DEST)

#
# Install Node.js modules
#

node_modules:
	@npm install

#
# Clean all
#

clean: clean-components clean-node

#
# Clean components & build
#

clean-components:
	@rm -rf build
	@rm -rf components

#
# Clean the installed Node.js modules
#

clean-node:
	@rm -rf node_modules

#
# Instructions
#

.PHONY: clean build components

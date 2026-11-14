#!/bin/bash

XPLANE_PLUGIN_DIR="$HOME/X-Plane 12/Resources/plugins/BetterPushback"

case "$(uname)" in
Linux)
    echo "copying lin_x64/BetterPushback.xpl to ${XPLANE_PLUGIN_DIR}"
	cp -r ./BetterPushback/lin_x64 "${XPLANE_PLUGIN_DIR}"
    echo "copying win_x64/BetterPushback.xpl to ${XPLANE_PLUGIN_DIR}"
	cp -r ./BetterPushback/win_x64 "${XPLANE_PLUGIN_DIR}"
	;;
Darwin)
    echo "copying mac_x64/BetterPushback.xpl to ${XPLANE_PLUGIN_DIR}"
	cp -r ./BetterPushback/mac_x64 "${XPLANE_PLUGIN_DIR}"
    echo "removing quarantine"
    xattr -dr com.apple.quarantine "${XPLANE_PLUGIN_DIR}"
	;;
*)
	echo "Unsupported platform" >&2
	exit 1
	;;
esac
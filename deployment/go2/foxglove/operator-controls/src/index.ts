import { ExtensionContext } from "@foxglove/extension";

import { initOperatorControls } from "./OperatorControls";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "operator-controls",
    initPanel: initOperatorControls,
  });
}

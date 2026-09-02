import { PanelExtensionContext, RenderState } from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

import "./styles.css";

type ActionId = "start-survey" | "stop-survey" | "stop-navigation";
type Tone = "primary" | "neutral" | "danger";

type Action = {
  id: ActionId;
  label: string;
  pendingLabel: string;
  serviceName: string;
  title: string;
  tone: Tone;
};

type Notice = {
  message: string;
  tone: "muted" | "success" | "error";
};

const ACTIONS: readonly Action[] = [
  {
    id: "start-survey",
    label: "START SURVEY",
    pendingLabel: "STARTING…",
    serviceName: "/navdp_go2_adapter/survey_start",
    title: "Start or resume RGB Survey recording",
    tone: "primary",
  },
  {
    id: "stop-survey",
    label: "STOP SURVEY",
    pendingLabel: "STOPPING…",
    serviceName: "/navdp_go2_adapter/survey_seal",
    title: "Stop recording and validate the Survey dataset",
    tone: "neutral",
  },
  {
    id: "stop-navigation",
    label: "STOP NAVIGATION",
    pendingLabel: "STOPPING…",
    serviceName: "/navdp_go2_adapter/operator_stop",
    title: "Disable motion, assert estop, and command zero",
    tone: "danger",
  },
] as const;

function responseNotice(action: Action, response: unknown): Notice {
  if (typeof response === "object" && response != undefined) {
    const result = response as { message?: unknown; success?: unknown };
    const message = typeof result.message === "string" ? result.message.trim() : "";
    if (result.success === false) {
      return {
        message: message || `${action.label} was rejected`,
        tone: "error",
      };
    }
  }
  return { message: `${action.label} complete`, tone: "success" };
}

function errorMessage(error: unknown): string {
  if (error instanceof Error && error.message.trim()) {
    return error.message;
  }
  return "Service call failed";
}

function ActionIcon({ action }: { action: ActionId }): ReactElement {
  if (action === "start-survey") {
    return (
      <svg aria-hidden="true" className="control-icon" viewBox="0 0 16 16">
        <path d="M5 3.5 12 8l-7 4.5z" fill="currentColor" />
      </svg>
    );
  }
  if (action === "stop-survey") {
    return (
      <svg aria-hidden="true" className="control-icon" viewBox="0 0 16 16">
        <rect x="4" y="4" width="8" height="8" rx="1" fill="currentColor" />
      </svg>
    );
  }
  return (
    <svg aria-hidden="true" className="control-icon" viewBox="0 0 16 16">
      <circle cx="8" cy="8" r="5.5" fill="none" stroke="currentColor" strokeWidth="1.5" />
      <rect x="5.25" y="5.25" width="5.5" height="5.5" rx="0.75" fill="currentColor" />
    </svg>
  );
}

function OperatorControls({ context }: { context: PanelExtensionContext }): ReactElement {
  const [colorScheme, setColorScheme] = useState<RenderState["colorScheme"]>();
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [pending, setPending] = useState<ReadonlySet<ActionId>>(() => new Set());
  const [notice, setNotice] = useState<Notice>({ message: "Ready", tone: "muted" });

  useLayoutEffect(() => {
    context.watch("colorScheme");
    context.onRender = (renderState, done) => {
      setColorScheme(renderState.colorScheme);
      setRenderDone(() => done);
    };
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const callAction = useCallback(
    async (action: Action): Promise<void> => {
      if (context.callService == undefined) {
        setNotice({ message: "Services unavailable for this connection", tone: "error" });
        return;
      }

      setPending((current) => new Set(current).add(action.id));
      setNotice({
        message: `${action.pendingLabel.replace("…", "").toLowerCase()}…`,
        tone: "muted",
      });
      try {
        const response = await context.callService(action.serviceName, {});
        setNotice(responseNotice(action, response));
      } catch (error) {
        setNotice({ message: errorMessage(error), tone: "error" });
      } finally {
        setPending((current) => {
          const next = new Set(current);
          next.delete(action.id);
          return next;
        });
      }
    },
    [context],
  );

  const servicesAvailable = context.callService != undefined;
  const status = servicesAvailable
    ? notice
    : { message: "Services unavailable for this connection", tone: "error" as const };

  return (
    <div className="operator-controls" data-color-scheme={colorScheme ?? "dark"}>
      <div className="control-row">
        {ACTIONS.map((action) => {
          const isPending = pending.has(action.id);
          return (
            <button
              key={action.id}
              aria-busy={isPending}
              className={`control-button control-button--${action.tone}`}
              disabled={!servicesAvailable || isPending}
              title={action.title}
              type="button"
              onClick={() => {
                void callAction(action);
              }}
            >
              <ActionIcon action={action.id} />
              <span>{isPending ? action.pendingLabel : action.label}</span>
            </button>
          );
        })}
      </div>
      <div
        aria-live="polite"
        className="control-status"
        data-tone={status.tone}
        title={status.message}
      >
        <span className="status-dot" />
        <span className="status-copy">{status.message}</span>
      </div>
    </div>
  );
}

export function initOperatorControls(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<OperatorControls context={context} />);
  return () => {
    root.unmount();
  };
}

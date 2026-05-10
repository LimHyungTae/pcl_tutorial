import { useState } from "react";
import { useT } from "../i18n";

export type BibEntry = {
  key: string;
  bibtex: string;
};

/** Renders one or more BibTeX entries as styled code blocks at the bottom of
 *  a chapter, with a per-entry copy button. Used by chapters whose
 *  algorithms have a canonical paper to cite. */
export default function BibTexBlock({ entries }: { entries: BibEntry[] }) {
  const t = useT();
  const [copiedKey, setCopiedKey] = useState<string | null>(null);

  const copy = async (e: BibEntry) => {
    try {
      await navigator.clipboard.writeText(e.bibtex);
      setCopiedKey(e.key);
      setTimeout(() => setCopiedKey((k) => (k === e.key ? null : k)), 1400);
    } catch {
      // Clipboard may be blocked in some browsers; fail silently — the user
      // can still select-and-copy the visible text.
    }
  };

  return (
    <section className="mt-10">
      <div className="code-font mb-2 text-[10px] font-semibold uppercase tracking-[0.2em] text-[var(--mut)]">
        {t.demo.bibtex}
      </div>
      <div className="space-y-3">
        {entries.map((e) => (
          <div key={e.key} className="relative">
            <button
              type="button"
              onClick={() => copy(e)}
              className="code-font absolute right-2 top-2 rounded border border-[var(--border-strong)] bg-[var(--surface)] px-2 py-1 text-[10px] text-[var(--dim)] transition hover:border-[var(--accent)] hover:text-[var(--accent)]"
            >
              {copiedKey === e.key ? t.demo.copied : t.demo.copy}
            </button>
            <pre className="code-font overflow-x-auto rounded-md border border-[var(--border)] bg-[var(--surface-2)] p-3 pr-20 text-[11px] leading-relaxed text-[var(--text)]">
              {e.bibtex}
            </pre>
          </div>
        ))}
      </div>
    </section>
  );
}

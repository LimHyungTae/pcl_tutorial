import { useLocale, type LocaleId } from "../i18n";

const labels: Record<LocaleId, string> = {
  en: "EN",
  ko: "한",
};

export default function LocaleToggle() {
  const { locale, setLocale } = useLocale();
  const order: LocaleId[] = ["en", "ko"];
  return (
    <div className="code-font inline-flex items-center overflow-hidden rounded-md border border-[var(--border-strong)] text-[11px]">
      {order.map((id) => (
        <button
          key={id}
          onClick={() => setLocale(id)}
          aria-pressed={locale === id}
          className={`px-2.5 py-1.5 transition ${
            locale === id
              ? "bg-[color:rgba(0,212,170,0.12)] text-[var(--accent)]"
              : "text-[var(--mut)] hover:text-[var(--text)]"
          }`}
        >
          {labels[id]}
        </button>
      ))}
    </div>
  );
}

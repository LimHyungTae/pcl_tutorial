import {
  createContext,
  ReactNode,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useState,
} from "react";
import { en, type LocaleDict } from "./locales/en";
import { ko } from "./locales/ko";

export type LocaleId = "en" | "ko";

const dicts: Record<LocaleId, LocaleDict> = { en, ko };

const STORAGE_KEY = "pcl-tutorial.locale";

type Ctx = {
  locale: LocaleId;
  setLocale: (l: LocaleId) => void;
  t: LocaleDict;
};

const LocaleContext = createContext<Ctx | null>(null);

export function LocaleProvider({ children }: { children: ReactNode }) {
  const [locale, setLocaleState] = useState<LocaleId>(() => {
    if (typeof window === "undefined") return "en";
    const stored = window.localStorage.getItem(STORAGE_KEY);
    return stored === "ko" ? "ko" : "en";
  });

  useEffect(() => {
    document.documentElement.lang = locale;
  }, [locale]);

  const setLocale = useCallback((l: LocaleId) => {
    setLocaleState(l);
    try {
      window.localStorage.setItem(STORAGE_KEY, l);
    } catch {
      /* private mode etc. */
    }
  }, []);

  const value = useMemo<Ctx>(
    () => ({ locale, setLocale, t: dicts[locale] }),
    [locale, setLocale],
  );

  return (
    <LocaleContext.Provider value={value}>{children}</LocaleContext.Provider>
  );
}

export function useT(): LocaleDict {
  const ctx = useContext(LocaleContext);
  if (!ctx) throw new Error("useT must be inside LocaleProvider");
  return ctx.t;
}

export function useLocale() {
  const ctx = useContext(LocaleContext);
  if (!ctx) throw new Error("useLocale must be inside LocaleProvider");
  return { locale: ctx.locale, setLocale: ctx.setLocale };
}

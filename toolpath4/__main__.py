"""Entry point for ``python -m toolpath4`` — launches the GUI."""

try:
    from toolpath4.gui import main
    main()
except ImportError as e:
    print(f"Cannot launch GUI: {e}")
    print("Make sure tkinter and matplotlib are installed.")
    raise SystemExit(1)

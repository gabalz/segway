
.phony: pdf clean

FILE="rategyro-model"

pdf:
	pdflatex ${FILE}.tex
	pdflatex ${FILE}.tex

cleangen:
	-rm -f *.dvi *.log *.aux *.out *.snm *.nav *.toc *.blg *.bbl

clean: cleangen
	-rm -f *.ps *.pdf


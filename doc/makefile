PDFLATEX = pdflatex

TARGET = $(shell ls *.tex|sed s/tex/pdf/)

%.pdf: %.tex
	$(PDFLATEX) $<
	$(PDFLATEX) $<

all: $(TARGET)
	@echo $(TARGET)

clean:
	@rm *.toc *.pdf *.aux *.log *.nav *.out *.snm *.vrb *~
